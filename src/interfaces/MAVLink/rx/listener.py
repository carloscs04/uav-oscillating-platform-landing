# file: src/interfaces/MAVLink/rx/listener.py
import time
import threading
from typing import Any, Dict, Optional
from src.interfaces.MAVLink.state.vehicle_state import VehicleState


class MAVLinkRXListener:
    """Non-blocking MAVLink listener tracking message throughput (Hz), watchdog, and critical Pixhawk params."""

    CRITICAL_PARAMS = [
        "PSC_NE_VEL_P",
        "PSC_NE_VEL_I",
        "PSC_NE_VEL_D",
        "PSC_NE_POS_P",
        "PSC_D_ACC_P",
        "PSC_D_VEL_P",
        "PSC_D_POS_P",
        # Optical flow + rangefinder navigation (checked by the 'flowcheck' command)
        "AHRS_EKF_TYPE",
        "EK3_ENABLE",
        "EK3_FLOW_DELAY",
        "EK3_FLOW_USE",
        "EK3_RNG_USE_HGT",
        "EK3_SRC1_POSXY",
        "EK3_SRC1_VELXY",
        "EK3_SRC1_POSZ",
        "EK3_SRC1_VELZ",
        "EK3_SRC1_YAW",
        "FLOW_TYPE",
        "FLOW_ORIENT_YAW",
        "FLOW_FXSCALER",
        "FLOW_FYSCALER",
        "VISO_TYPE",
        "RNGFND1_TYPE",
        "RNGFND1_ORIENT",
        "RNGFND1_MIN",
        "RNGFND1_MAX",
        "RNGFND1_MIN_CM",
        "RNGFND1_MAX_CM",
        "COMPASS_ENABLE",
        "COMPASS_USE",
        "COMPASS_USE2",
        "COMPASS_USE3",
        # Safety parameters that debug_force_arm overwrites (must be restored before flight)
        "ARMING_CHECK",
        "ARMING_SKIPCHK",
        "FS_THR_ENABLE",
        "DISARM_DELAY",
        "BRD_SAFETYENABLE",
        "BRD_SAFETY_DEFLT",
    ]

    def __init__(self, state: VehicleState):
        self.state = state
        self.master: Optional[Any] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.last_recv_time: float = 0.0

        # Throughput tracking (Hz)
        self._msg_counter: int = 0
        self._last_rate_calc: float = time.time()
        self.current_rate_hz: float = 0.0

        # Non-blocking parameter fetch state
        self._param_queue_idx: int = 0
        self._last_param_req_time: float = 0.0
        self._param_sync_rounds: int = 0

    def set_master(self, master: Any):
        self.master = master
        self.last_recv_time = time.time()
        self._msg_counter = 0
        self._param_queue_idx = 0
        self._param_sync_rounds = 0

    def wait_for_heartbeat(self, timeout: float = 5.0) -> Optional[Dict[str, Any]]:
        if not self.master:
            return None
        deadline = time.time() + timeout
        try:
            while time.time() < deadline:
                hb = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
                if hb and hb.get_srcSystem() == 1 and hb.get_srcComponent() == 1:
                    self.master.target_system = 1
                    self.master.target_component = 1
                    self.last_recv_time = time.time()
                    return {"system_id": 1}
        except Exception:
            pass
        return None

    def start(self):
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self.last_recv_time = time.time()
        self._last_rate_calc = time.time()
        self._msg_counter = 0
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=0.2)
        self._thread = None
        self.master = None

    def refresh_params(self):
        """Re-read every CRITICAL_PARAMS entry from the Pixhawk (non-blocking)."""
        self._param_queue_idx = 0
        self._param_sync_rounds = 0

    def _poll_next_critical_param(self):
        """Requests one critical parameter at a time without blocking the RX thread."""
        if not self.master or self._param_sync_rounds >= 2:
            return

        now = time.time()
        if now - self._last_param_req_time < 0.06:
            return

        if self._param_queue_idx < len(self.CRITICAL_PARAMS):
            param_name = self.CRITICAL_PARAMS[self._param_queue_idx]
            try:
                self.master.mav.param_request_read_send(
                    1,
                    1,
                    param_name.encode("utf-8"),
                    -1
                )
            except Exception:
                pass
            self._param_queue_idx += 1
            self._last_param_req_time = now
        else:
            # Second pass to catch any dropped UDP/serial packet
            self._param_queue_idx = 0
            self._param_sync_rounds += 1

    def _read_loop(self):
        while not self._stop_event.is_set() and self.master:
            try:
                self._poll_next_critical_param()

                # Drain all queued MAVLink packets
                while True:
                    msg = self.master.recv_match(blocking=False)
                    if not msg:
                        break

                    # Ignore packets not originating from Pixhawk autopilot (sysid=1, compid=1)
                    if msg.get_srcSystem() != 1 or msg.get_srcComponent() != 1:
                        continue

                    data = msg.to_dict()
                    pkt_type = data.get("mavpackettype")

                    if pkt_type and pkt_type != "BAD_DATA":
                        self.last_recv_time = time.time()
                        self._msg_counter += 1

                        if pkt_type == "PARAM_VALUE":
                            raw_id = data.get("param_id", "")
                            if isinstance(raw_id, bytes):
                                raw_id = raw_id.decode("utf-8", errors="ignore")
                            param_id = str(raw_id).strip("\x00").strip()
                            param_val = data.get("param_value")
                            if param_id and param_val is not None:
                                self.state.update_param(param_id, round(float(param_val), 4))
                        else:
                            self.state.update_packet(pkt_type, data)

                # Compute throughput in Hz every 1.0 second
                now = time.time()
                elapsed = now - self._last_rate_calc
                if elapsed >= 1.0:
                    self.current_rate_hz = round(self._msg_counter / elapsed, 1)
                    self._msg_counter = 0
                    self._last_rate_calc = now

            except Exception:
                time.sleep(0.05)

            self._stop_event.wait(0.005)