# src/main.py
import time
from interfaces.hardware.mavlink.manager import MAVLinkLauncher

def boot():
    print("=== Testing MAVLink Connection with PX4 SITL ===")
    
    # 1. Initialize launcher
    launcher = MAVLinkLauncher()
    
    # 2. Attempt connection to SITL
    result = launcher.start_connection()

    if not result["success"]:
        print(f"❌ Connection Failed [{result['status_code']}]: {result['message']}")
        return

    print(f"✅ Connection Established [{result['status_code']}]: {result['message']}")
    
    # 3. Access driver instance
    driver = result["driver"]
    
    try:
        print("\nStreaming local position from SITL (5 seconds)...")
        for _ in range(10):
            pos = driver.master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1.0)
            if pos:
                print(f"Position -> X: {pos.x:.2f}m, Y: {pos.y:.2f}m, Z: {pos.z:.2f}m")
            time.sleep(0.5)
    finally:
        driver.close()

if __name__ == "__main__":
    boot()