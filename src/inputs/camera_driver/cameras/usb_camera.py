"""USB (UVC) camera backend using OpenCV's V4L2 capture."""

import glob
import os

import cv2

from .base import Camera, CameraError


def find_video_node(vid_pid):
    """
    Return the /dev/videoN capture node of the USB camera with this
    'vendor:product' ID. /dev/videoN numbers can change between boots;
    the USB ID does not.

    A UVC webcam creates two nodes (images + metadata); the image node is
    the one whose sysfs 'index' is 0.
    """
    vid_pid = vid_pid.lower()
    for node in sorted(glob.glob("/sys/class/video4linux/video*")):
        usb_dir = os.path.join(node, "device", "..")
        try:
            vid = open(os.path.join(usb_dir, "idVendor")).read().strip()
            pid = open(os.path.join(usb_dir, "idProduct")).read().strip()
            index = open(os.path.join(node, "index")).read().strip()
        except OSError:
            continue  # not a USB device (Pi ISP, codec, CSI receiver, ...)
        if f"{vid}:{pid}" == vid_pid and index == "0":
            return "/dev/" + os.path.basename(node)
    raise CameraError(f"USB camera {vid_pid} not found (is it plugged in?)")


class UsbCamera(Camera):
    backend = "usb"

    def __init__(self, camera_id, resolution, fps=30.0, vid_pid=None,
                 device=None, fourcc="MJPG", **kwargs):
        super().__init__(camera_id, resolution, fps, **kwargs)
        if not vid_pid and not device:
            raise CameraError(f"{camera_id}: set 'vid_pid' (preferred) or 'device'")
        self.vid_pid = vid_pid
        self.device = device
        self.fourcc = fourcc
        self._cap = None
        self._device_path = None

    def _open(self):
        self._device_path = self.device or find_video_node(self.vid_pid)
        cap = cv2.VideoCapture(self._device_path, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise CameraError(f"{self.camera_id}: cannot open {self._device_path}")

        # FOURCC must be set before the size: many webcams only offer
        # high resolutions in MJPG.
        if self.fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.resolution[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.resolution[1])
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        # 2 buffers: frames are at most one frame old. With 1 buffer the
        # driver has no free buffer while we hold one and drops every other
        # frame (measured 17.6 FPS instead of 30 on the 0bdc:8088 webcam).
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 2)
        # Autofocus would change the lens geometry and invalidate the
        # calibration. Ignored by cameras without autofocus.
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        self._cap = cap

    def _read(self):
        ok, frame = self._cap.read()
        return frame if ok else None

    def _close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def info(self):
        d = super().info()
        d.update({"vid_pid": self.vid_pid, "device": self._device_path,
                  "fourcc": self.fourcc})
        return d
