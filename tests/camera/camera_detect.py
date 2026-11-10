"""
camera_detect.py
Detects USB (UVC) and CSI (libcamera / Picamera2) cameras on a Raspberry Pi
and optionally grabs a test frame from each one.

Usage:
    python3 camera_detect.py          # detect + save test images
    In Jupyter:  %run camera_detect.py
                 or: from camera_detect import detect_all
"""

import glob
import os

import cv2

# V4L2 nodes created by the Pi's own hardware (ISP, codecs, CSI receiver).
# They appear as /dev/videoX but are NOT USB cameras, so we skip them.
PI_INTERNAL_NODES = ("bcm2835", "unicam", "rp1-cfe", "pispbe", "hevc")


# ---------------------------------------------------------------------------
# USB cameras (OpenCV / V4L2)
# ---------------------------------------------------------------------------
def _v4l2_name(dev_path):
    """Read the driver-reported name of a /dev/videoX node."""
    node = os.path.basename(dev_path)
    try:
        with open(f"/sys/class/video4linux/{node}/name") as f:
            return f.read().strip()
    except OSError:
        return "unknown"


def list_usb_cameras(test_open=True):
    """
    Return a list of usable USB cameras.

    A UVC webcam usually creates two nodes (e.g. /dev/video0 for images and
    /dev/video1 for metadata). With test_open=True only nodes that actually
    return a frame are kept, which removes the metadata node.
    """
    cameras = []
    devices = sorted(glob.glob("/dev/video*"),
                     key=lambda p: int(p.replace("/dev/video", "")))

    for dev in devices:
        name = _v4l2_name(dev)
        if any(tag in name.lower() for tag in PI_INTERNAL_NODES):
            continue

        index = int(dev.replace("/dev/video", ""))
        info = {"type": "usb", "index": index, "device": dev, "name": name}

        if test_open:
            cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
            ok = False
            if cap.isOpened():
                ok, _ = cap.read()
                if ok:
                    info["width"] = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                    info["height"] = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                    info["fps"] = cap.get(cv2.CAP_PROP_FPS)
            cap.release()
            if not ok:
                continue

        cameras.append(info)

    return cameras


def grab_usb_frame(index, width=1280, height=720):
    """Capture a single BGR frame from a USB camera."""
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    # Discard a few frames so auto-exposure can settle
    for _ in range(5):
        cap.read()
    ok, frame = cap.read()
    cap.release()
    return frame if ok else None


# ---------------------------------------------------------------------------
# CSI cameras (libcamera / Picamera2)
# ---------------------------------------------------------------------------
def list_csi_cameras():
    """Return a list of CSI cameras detected by libcamera."""
    try:
        from picamera2 import Picamera2
    except ImportError:
        print("[CSI] picamera2 not available. Install with: "
              "sudo apt install python3-picamera2")
        return []

    cameras = []
    for i, cam in enumerate(Picamera2.global_camera_info()):
        cam_id = cam.get("Id", "")
        # libcamera can also list USB webcams; keep only CSI sensors
        if "usb" in cam_id.lower():
            continue
        cameras.append({
            "type": "csi",
            "index": cam.get("Num", i),
            "name": cam.get("Model", "unknown"),  # e.g. "imx219"
            "id": cam_id,
        })
    return cameras


def grab_csi_frame(index, width=1280, height=720):
    """Capture a single BGR frame from a CSI camera."""
    from picamera2 import Picamera2

    picam = Picamera2(index)
    # "RGB888" in Picamera2 is stored in BGR order, ready for OpenCV
    config = picam.create_preview_configuration(
        main={"size": (width, height), "format": "RGB888"})
    picam.configure(config)
    picam.start()
    for _ in range(5):          # let auto-exposure settle
        picam.capture_array()
    frame = picam.capture_array()
    picam.stop()
    picam.close()
    return frame


# ---------------------------------------------------------------------------
# Combined detection
# ---------------------------------------------------------------------------
def detect_all(verbose=True):
    """Detect every camera and return {'usb': [...], 'csi': [...]}."""
    found = {"usb": list_usb_cameras(), "csi": list_csi_cameras()}

    if verbose:
        print("=== Camera detection ===")
        if not found["usb"] and not found["csi"]:
            print("No cameras found.")
        for cam in found["usb"]:
            print(f"[USB] index={cam['index']}  {cam['device']}  "
                  f"name='{cam['name']}'  "
                  f"{cam.get('width')}x{cam.get('height')} @ {cam.get('fps')} FPS")
        for cam in found["csi"]:
            print(f"[CSI] index={cam['index']}  model='{cam['name']}'")

    return found


def save_test_frames(found, out_dir=None):
    """
    Grab one frame from every detected camera and save it as a JPG.
    Default folder: tests/camera/test_frames, wherever the script is run from.
    """
    if out_dir is None:
        out_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "test_frames")
    os.makedirs(out_dir, exist_ok=True)

    for cam in found["usb"]:
        frame = grab_usb_frame(cam["index"])
        _save(frame, os.path.join(out_dir, f"usb_{cam['index']}.jpg"))

    for cam in found["csi"]:
        frame = grab_csi_frame(cam["index"])
        _save(frame, os.path.join(out_dir, f"csi_{cam['name']}_{cam['index']}.jpg"))


def _save(frame, path):
    if frame is None:
        print(f"  Could not capture frame for {path}")
        return
    cv2.imwrite(path, frame)
    h, w = frame.shape[:2]
    print(f"  Saved {path} ({w}x{h})")


if __name__ == "__main__":
    cams = detect_all()
    print("\n=== Test capture ===")
    save_test_frames(cams)