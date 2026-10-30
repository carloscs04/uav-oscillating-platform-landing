import time
from src.core.drone_state import DroneState
from src.modules.vision.tracker import TargetTracker
from src.modules.control.pid_controller import PlatformTrackingController

def main():
    print("==================================================")
    print(" Starting UAV OPL Pure Python Testing Engine")
    print("==================================================")
    
    state = DroneState(timestamp=time.time(), x=0.0, y=0.0, z=3.0, armed=True)
    tracker = TargetTracker()
    controller = PlatformTrackingController(kp_xy=0.8, kp_z=0.5)

    # Simulated loop execution without ROS
    target = tracker.process_frame(raw_data=b"mock_frame_bytes", width=640, height=480)
    cmd = controller.compute_velocities(
        drone_z=state.z,
        target_x=target['x'],
        target_y=target['y']
    )

    print(f"[STATE] Drone Altitude: {state.z}m")
    print(f"[VISION] Relative Target Position: {target}")
    print(f"[CONTROL] Calculated Output Setpoints: {cmd}")
    print("==================================================")

if __name__ == "__main__":
    main()