"""CSI ribbon camera backend using Picamera2 (libcamera).

Picamera2 is installed with apt (python3-picamera2), not pip.
"""

from .base import Camera, CameraError


def find_csi_index(model):
    """
    Return the libcamera index of the CSI sensor with this model name
    (e.g. 'imx219'). libcamera also lists USB webcams, so the index of a
    CSI camera depends on what else is plugged in; the model name does not.
    """
    from picamera2 import Picamera2

    found = []
    for info in Picamera2.global_camera_info():
        if "usb" in info.get("Id", "").lower():
            continue
        found.append(info.get("Model", "?"))
        if info.get("Model", "").lower() == model.lower():
            return info["Num"]
    raise CameraError(f"CSI camera '{model}' not found. Detected CSI cameras: {found or 'none'}")


class CsiCamera(Camera):
    backend = "csi"

    def __init__(self, camera_id, resolution, fps=30.0, model=None,
                 sensor_mode=None, controls=None, tuning_file=None, **kwargs):
        super().__init__(camera_id, resolution, fps, **kwargs)
        if not model:
            raise CameraError(f"{camera_id}: 'model' is required for CSI cameras")
        self.model = model
        self.sensor_mode = tuple(sensor_mode) if sensor_mode else None
        self.controls = dict(controls or {})
        # e.g. "imx219_noir.json" for sensors without an IR-cut filter;
        # None uses libcamera's default tuning for the sensor.
        self.tuning_file = tuning_file
        self._picam = None
        self._sensor_info = {}
        self._software_rotate = False  # rotation done by the ISP (free)

    def _open(self):
        from libcamera import Transform
        from picamera2 import Picamera2

        index = find_csi_index(self.model)
        tuning = Picamera2.load_tuning_file(self.tuning_file) if self.tuning_file else None
        picam = Picamera2(index, tuning=tuning)
        try:
            kwargs = {
                # "RGB888" in Picamera2 is stored in BGR order, ready for OpenCV.
                "main": {"size": self.resolution, "format": "RGB888"},
                "controls": {"FrameRate": self.fps, **self.controls},
                "buffer_count": 4,
            }
            # Choosing the sensor mode explicitly matters: Picamera2 may
            # otherwise pick a cropped mode, which changes the field of view
            # and invalidates the calibration.
            if self.sensor_mode:
                kwargs["sensor"] = {"output_size": self.sensor_mode}
            if self.rotate_180:
                kwargs["transform"] = Transform(hflip=1, vflip=1)

            picam.configure(picam.create_video_configuration(**kwargs))

            # Lock focus on autofocus sensors (e.g. imx708); same reason as above.
            if "AfMode" in picam.camera_controls:
                from libcamera import controls as lc
                picam.set_controls({"AfMode": lc.AfModeEnum.Manual})

            picam.start()
        except Exception:
            picam.close()
            raise

        sensor = picam.camera_configuration().get("sensor", {})
        self._sensor_info = {k: (list(v) if isinstance(v, tuple) else v)
                             for k, v in sensor.items()}
        if self.sensor_mode and tuple(sensor.get("output_size", ())) != self.sensor_mode:
            picam.stop()
            picam.close()
            raise CameraError(
                f"{self.camera_id}: sensor mode {self.sensor_mode} requested but "
                f"libcamera chose {sensor.get('output_size')}")
        self._picam = picam

    def _read(self):
        return self._picam.capture_array("main")

    def _close(self):
        if self._picam is not None:
            try:
                self._picam.stop()
            finally:
                self._picam.close()
                self._picam = None

    def info(self):
        d = super().info()
        d.update({"model": self.model, "sensor": self._sensor_info,
                  "tuning_file": self.tuning_file or "default"})
        return d
