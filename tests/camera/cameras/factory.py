"""Build the configured camera backend from config.yaml."""

import logging
from pathlib import Path

import yaml

from .base import CameraError
from .csi_camera import CsiCamera
from .usb_camera import UsbCamera

log = logging.getLogger(__name__)

DEFAULT_CONFIG = Path(__file__).resolve().parent.parent / "config.yaml"

BACKENDS = {
    "usb": UsbCamera,
    "csi": CsiCamera,
}


def load_config(path=None):
    with open(path or DEFAULT_CONFIG) as f:
        return yaml.safe_load(f)


def get_camera(name=None, config=None):
    """
    Return an (unopened) Camera. `name` defaults to `active_camera`.
    `config` may be a dict, a path, or None for the default config.yaml.
    """
    if not isinstance(config, dict):
        config = load_config(config)
    name = name or config["active_camera"]
    cameras = config.get("cameras", {})
    if name not in cameras:
        raise CameraError(f"camera '{name}' is not in the config. Options: {list(cameras)}")

    params = dict(cameras[name])
    backend = params.pop("backend")
    if backend not in BACKENDS:
        raise CameraError(f"{name}: unknown backend '{backend}'. Options: {list(BACKENDS)}")
    return BACKENDS[backend](camera_id=name, **params)


def open_camera(name=None, config=None, fallback=None):
    """
    Return an opened Camera. If it fails and fallback is enabled (argument,
    or `fallback:` in the config), try the other configured cameras in order.
    """
    if not isinstance(config, dict):
        config = load_config(config)
    name = name or config["active_camera"]
    if fallback is None:
        fallback = config.get("fallback", False)

    candidates = [name]
    if fallback:
        candidates += [n for n in config.get("cameras", {}) if n != name]

    errors = []
    for candidate in candidates:
        try:
            cam = get_camera(candidate, config).open()
        except Exception as exc:  # noqa: BLE001 - report every failure
            errors.append(f"{candidate}: {exc}")
            log.warning("camera %s failed to open: %s", candidate, exc)
            continue
        if candidate != name:
            log.warning("using fallback camera %s instead of %s", candidate, name)
        return cam

    raise CameraError("no camera could be opened:\n  " + "\n  ".join(errors))
