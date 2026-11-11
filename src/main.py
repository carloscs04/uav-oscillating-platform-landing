# File: src/main.py

import math
import random
import sys
import time

# 1. MAVLink e Interfaces Telecom
from src.interfaces.Bridge_WIFI.master_wifi import MasterWIFI
from src.interfaces.MAVLink.master_mavlink import MAVLinkManager

# 2. Entradas (Drivers/Sensores State)
from src.inputs.camera_driver.camera_state import CameraState
from src.inputs.distance_sensor.sensor_state import DistanceSensorState

# 3. Lógica de Control y Nodo Maestro (State)
from src.logic.control.control_state import ControlState
from src.logic.master.master_state import MasterState

# ==============================================================================
# ⚠️ MODO SIMULACIÓN DUMMY EN MAIN
# Cambia DUMMY_MODE a False para volver al comportamiento real sin datos falsos.
# ==============================================================================
DUMMY_MODE = True


def _patch_dummy_telemetry(camera_state, distance_state, control_state, master_state):
    """Parchea dinámicamente los métodos export en los objetos State

    para emitir datos dinámicos en todas las pestañas de la GCS.
    """
    t_start = time.time()

    # --- SIMULACIÓN CONTROL NODE ([control_state], [pid_output], [status]) ---
    def dummy_control_telemetry():
        t = time.time() - t_start
        act_x = round(0.15 * math.sin(t * 2.0) + random.uniform(-0.01, 0.01), 3)
        act_y = round(0.12 * math.cos(t * 2.0) + random.uniform(-0.01, 0.01), 3)
        act_z = round(1.20 + 0.10 * math.sin(t * 1.0) + random.uniform(-0.01, 0.01), 3)

        return {
            "control_state": {
                "ref_x": 0.0, "ref_y": 0.0, "ref_z": 1.2,
                "act_x": act_x, "act_y": act_y, "act_z": act_z,
                "error_x": round(0.0 - act_x, 3),
                "error_y": round(0.0 - act_y, 3),
                "error_z": round(1.2 - act_z, 3)
            },
            "pid_output": {
                "p_term": round(act_x * 0.5, 3),
                "i_term": round(act_y * 0.1, 3),
                "d_term": round(random.uniform(-0.02, 0.02), 3),
                "total_cmd": round(act_x * 0.5 + act_y * 0.1, 3)
            },
            "status": {
                "control_mode": "AUTO_DESCENT",
                "armed": True,
                "is_stable": True
            }
        }

    def dummy_control_health(rate_hz=20.0, is_connected=True):
        return {"status": "HEALTHY", "rate_hz": rate_hz, "battery_pct": 95, "voltage_v": 12.4}

    # --- SIMULACIÓN DISTANCE SENSOR ([distance_sensor], [status]) ---
    def dummy_distance_telemetry():
        t = time.time() - t_start
        dist = round(1.5 + 0.25 * math.sin(t * 1.5) + random.uniform(-0.01, 0.01), 3)
        return {
            "distance_sensor": {"distance_m": dist, "sensor_type": "LiDAR_ToF"},
            "status": {"valid": True, "signal_quality": 99}
        }

    def dummy_distance_health(rate_hz=15.0, is_connected=True):
        return {"status": "HEALTHY", "rate_hz": rate_hz, "battery_pct": 98, "voltage_v": 5.0}

    # --- SIMULACIÓN CAMERA & ARUCO ([aruco_pose], [video_frame], [status]) ---
    def dummy_camera_telemetry():
        t = time.time() - t_start
        x_val = round(0.08 * math.sin(t * 1.8), 3)
        y_val = round(-0.05 * math.cos(t * 1.8), 3)
        z_val = round(1.25 + random.uniform(-0.01, 0.01), 3)
        return {
            "aruco_pose": {"detected": True, "id": 42, "x": x_val, "y": y_val, "z": z_val},
            "video_frame": {"fps": 30.0, "width": 640, "height": 480, "stream": "ACTIVE"},
            "status": {"camera_online": True, "exposure": "AUTO"}
        }

    def dummy_camera_health(rate_hz=30.0, is_connected=True):
        return {"status": "HEALTHY", "rate_hz": rate_hz, "battery_pct": 90, "voltage_v": 5.1}

    # --- SIMULACIÓN MASTER NODE ([master_status], [status]) ---
    def dummy_master_telemetry():
        return {
            "master_status": {
                "cpu_usage": round(15.0 + random.uniform(-2.0, 4.0), 1),
                "ram_usage": round(42.0 + random.uniform(-0.5, 0.5), 1),
                "uptime_sec": int(time.time() - t_start)
            },
            "status": {"system_state": "NOMINAL", "bridge_wifi": "CONNECTED"}
        }

    def dummy_master_health(rate_hz=10.0, is_connected=True):
        return {"status": "HEALTHY", "rate_hz": rate_hz, "battery_pct": 100, "voltage_v": 12.0}

    # Asignar métodos dummy a los objetos State
    control_state.export_telemetry_data = dummy_control_telemetry
    control_state.export_node_health = dummy_control_health

    distance_state.export_telemetry_data = dummy_distance_telemetry
    distance_state.export_node_health = dummy_distance_health

    camera_state.export_telemetry_data = dummy_camera_telemetry
    camera_state.export_node_health = dummy_camera_health

    master_state.export_telemetry_data = dummy_master_telemetry
    master_state.export_node_health = dummy_master_health


def main():
    mavlink_node = None
    bridge_wifi = None

    print("[MAIN] Starting self-healing subsystem nodes...")

    # 1. Instanciar MAVLink y sub-nodos de estado
    mavlink_node = MAVLinkManager()
    mavlink_node.start(mode="HARDWARE")

    camera_state = CameraState()
    distance_state = DistanceSensorState()
    control_state = ControlState()
    master_state = MasterState()

    # 🔗 FIX CLAVE: Hacemos compatibles los objetos State con los transmisores (tx/*.py)
    camera_state.state = camera_state
    distance_state.state = distance_state
    control_state.state = control_state
    master_state.state = master_state

    # Atributos auxiliares de consulta
    for state_obj in (camera_state, distance_state, control_state, master_state):
        setattr(state_obj, "connected", True)
        setattr(state_obj, "current_rate_hz", 20.0)

    # 2. Inyectar datos dummy si está activo
    if DUMMY_MODE:
        print("[MAIN] ⚠️ Running with DUMMY TELEMETRY PATCH enabled for GCS testing...")
        _patch_dummy_telemetry(camera_state, distance_state, control_state, master_state)

    try:
        print("[MAIN] Starting MasterWIFI telecom bridge...")

        # 3. Iniciar MasterWIFI
        bridge_wifi = MasterWIFI(
            handshake_port=5007,
            mavlink_node=mavlink_node,
            camera_node=camera_state,
            distance_node=distance_state,
            control_node=control_state,
            master_node=master_state
        )

        bridge_wifi.start()

    except KeyboardInterrupt:
        print("\n[MAIN] KeyboardInterrupt detected. Stopping...")
    except Exception as e:
        print(f"\n[MAIN Error] Unexpected system execution error: {e}")
    finally:
        print("[MAIN] Safely terminating all active onboard subsystem nodes...")
        if bridge_wifi is not None:
            try:
                bridge_wifi.stop()
                print("[MAIN] MasterWIFI telecom bridge stopped cleanly.")
            except Exception as e:
                print(f"[MAIN Warning] Exception during MasterWIFI shutdown: {e}")

        if mavlink_node is not None:
            try:
                mavlink_node.stop()
                print("[MAIN] MAVLink hardware manager node stopped cleanly.")
            except Exception as e:
                print(f"[MAIN Warning] Exception during MAVLink shutdown: {e}")

        print("[MAIN] All onboard nodes terminated safely. System offline.")


if __name__ == "__main__":
    main()