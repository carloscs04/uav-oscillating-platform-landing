"""
gcs_video_link.py - stream the real camera to the ground station, without MAVLink.

Runs only:  CameraManager         (src/inputs/camera_driver/master_camera.py)  -> CameraState
            VisionManager         (src/logic/vision/master_vision.py)          -> VisionState
            DistanceSensorManager (src/inputs/distance_sensor/master_sensor.py) -> DistanceSensorState
            MasterWIFI  (Bridge_WIFI: camera 5003, vision 5006, distance 5002)

Use it to check the video (and ArUco pose) reach the ground station panel when
the full src/main.py cannot run (e.g. pymavlink missing, no flight controller).

    cd ~/uav-opl
    python3 tests/vision/gcs_video_link.py              # camera + ArUco
    python3 tests/vision/gcs_video_link.py --no-vision  # video only
    python3 tests/vision/gcs_video_link.py --no-distance  # without the VL53L1X

Then on the laptop set the ground-station profile's pi_ip to one of the IPs
printed below and start the panel. Ctrl+C to stop.
"""

import argparse
import socket
import subprocess
import threading
import time

import _paths  # noqa: F401  (repo root on sys.path)
from src.inputs.camera_driver.master_camera import CameraManager
from src.inputs.distance_sensor.master_sensor import DistanceSensorManager
from src.interfaces.Bridge_WIFI.master_wifi import MasterWIFI
from src.logic.vision.master_vision import VisionManager


def local_ips():
    """IPv4 address of every network interface (name -> ip)."""
    try:
        out = subprocess.run(["ip", "-4", "-br", "addr"], capture_output=True, text=True).stdout
    except OSError:
        return {socket.gethostname(): "?"}
    ips = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[0] != "lo":
            ips[parts[0]] = parts[2].split("/")[0]
    return ips


def status_loop(camera, vision, distance, bridge, stop):
    while not stop.is_set():
        stop.wait(2.0)
        cam = camera.export_telemetry_package()
        gcs = bridge.target_ip or "waiting for handshake on :5007"
        line = (f"[LINK] GCS={gcs} | camera {camera.calib_key} {cam['node_health']['status']} "
                f"{cam['node_health']['rate_hz']} Hz, preview {len(cam['video_frame']['data']) // 1024} KB b64")
        if vision is not None:
            vis = vision.export_telemetry_package()
            p, st = vis["aruco_pose"], vis["status"]
            target = (f"ID {p['id']} x{p['x']:+.3f} y{p['y']:+.3f} z{p['z']:.3f} "
                      f"d={p['distance_m']:.3f} m {st['tracking_state']}"
                      if p["detected"] else st["tracking_state"])
            line += f" | vision {vis['node_health']['status']}: {target}"
        if distance is not None:
            dis = distance.export_telemetry_package()
            d = dis["distance_sensor"]
            line += (f" | distance {dis['node_health']['status']} "
                     f"{dis['node_health']['rate_hz']} Hz: {d['distance_m']:.3f} m {d['status']}")
        print(line)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-vision", action="store_true", help="stream video only")
    ap.add_argument("--no-distance", action="store_true", help="do not start the distance sensor")
    ap.add_argument("--camera", help="camera name from config.yaml (default: active_camera)")
    args = ap.parse_args()

    camera = CameraManager(camera=args.camera)
    vision = None if args.no_vision else VisionManager(camera)
    distance = None if args.no_distance else DistanceSensorManager()
    camera.start(timeout=0)
    if vision:
        vision.start(timeout=0)
    if distance:
        distance.start(timeout=0)

    print("Put one of these in the ground-station profile as pi_ip:")
    for name, ip in local_ips().items():
        print(f"   {name:8s} {ip}")

    bridge = MasterWIFI(handshake_port=5007, camera_node=camera.state,
                        vision_node=vision.state if vision else None,
                        distance_node=distance.state if distance else None)
    stop = threading.Event()
    threading.Thread(target=status_loop, args=(camera, vision, distance, bridge, stop), daemon=True).start()
    try:
        bridge.start()            # blocks until Ctrl+C (then stops itself)
    except KeyboardInterrupt:
        bridge.stop()
    finally:
        stop.set()
        if vision:
            vision.stop()
        if distance:
            distance.stop()
        camera.stop()
        time.sleep(0.2)
        print("Stopped.")


if __name__ == "__main__":
    main()
