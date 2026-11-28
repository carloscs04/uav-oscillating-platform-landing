"""Upload a Lua script to the Pixhawk SD card (APM/scripts/) over MAVLink FTP, no SD removal.

The onboard service owns the serial port, so stop it first (DISARMED, on the ground):
    sudo systemctl stop uav-opl.service
    .venv/bin/python tools/pixhawk_put_script.py src/interfaces/MAVLink/scripts/plnd_probe.lua
    sudo systemctl start uav-opl.service

Options: --rm NAME removes APM/scripts/NAME instead. After the upload the Pixhawk's scripting is
restarted (MAV_CMD_SCRIPTING stop-and-restart), so the script runs without a reboot.
The port and baud come from src/interfaces/MAVLink/config.yaml (HARDWARE)."""
import argparse
import os
import sys
import time

import yaml
from pymavlink import mavutil
from pymavlink.mavftp import MAVFTP, FtpError

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
REMOTE_DIR = "APM/scripts"


def hardware_target():
    with open(os.path.join(REPO, "src", "interfaces", "MAVLink", "config.yaml")) as f:
        hw = yaml.safe_load(f)["mavlink"]["targets"]["HARDWARE"]
    return hw["uri"], int(hw.get("baud_rate", 921600))


def connect(uri: str, baud: int):
    master = mavutil.mavlink_connection(uri, baud=baud, source_system=255)
    print(f"waiting for heartbeat on {uri} ...")
    t_end = time.time() + 10
    while True:   # the autopilot's heartbeat, not a sysid-0 / GCS one (that left target 0 = broadcast)
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=max(0.1, t_end - time.time()))
        if hb is None:
            sys.exit("no heartbeat: is uav-opl.service still running (port busy) or the cable loose?")
        if hb.get_srcSystem() != 0 and hb.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID:
            master.target_system, master.target_component = hb.get_srcSystem(), hb.get_srcComponent()
            break
    print(f"connected: system {master.target_system}, component {master.target_component}")
    return master


def restart_scripting(master):
    master.mav.command_long_send(master.target_system, master.target_component,
                                 mavutil.mavlink.MAV_CMD_SCRIPTING, 0,
                                 mavutil.mavlink.SCRIPTING_CMD_STOP_AND_RESTART, 0, 0, 0, 0, 0, 0)
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    result = getattr(ack, "result", None)
    print("scripting restart:", "accepted" if result == 0 else f"result {result} (reboot the Pixhawk instead)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file", nargs="?", help="local .lua file to upload")
    ap.add_argument("--rm", help="remove APM/scripts/<name> from the SD card")
    ap.add_argument("--no-restart", action="store_true", help="do not restart scripting after upload")
    args = ap.parse_args()
    if not args.file and not args.rm:
        ap.error("give a file to upload or --rm NAME")

    uri, baud = hardware_target()
    master = connect(uri, baud)
    ftp = MAVFTP(master, master.target_system, master.target_component)

    ftp.cmd_mkdir(["APM"])            # "exists" errors are fine
    ftp.cmd_mkdir([REMOTE_DIR])

    if args.rm:
        ret = ftp.cmd_rm([f"{REMOTE_DIR}/{args.rm}"])
        print("remove:", "ok" if ret.error_code == FtpError.Success else "failed")
        ret.display_message()
    else:
        name = os.path.basename(args.file)
        t0 = time.time()
        ret = ftp.cmd_put([args.file, f"{REMOTE_DIR}/{name}"])
        if ret.error_code == FtpError.Success:
            ret = ftp.process_ftp_reply("put", timeout=60)
        if ret.error_code != FtpError.Success:
            ret.display_message()
            master.close()
            sys.exit(f"upload of {name} failed")
        print(f"uploaded {name} ({os.path.getsize(args.file)} bytes) in {time.time() - t0:.1f} s")

    ret = ftp.cmd_list([REMOTE_DIR])
    print(f"{REMOTE_DIR}/ now has:")
    for e in (ret.directory_listing or []):
        print(f"   {e.name}  {'' if e.is_dir else e.size_b}")

    if not args.no_restart:
        restart_scripting(master)
    master.close()


if __name__ == "__main__":
    main()
