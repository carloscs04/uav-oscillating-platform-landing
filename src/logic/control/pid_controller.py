class PlatformTrackingController:
    """Pure Python controller for tracking and descending onto an oscillating platform."""
    
    def __init__(self, kp_xy: float = 0.8, kp_z: float = 0.5):
        self.kp_xy = kp_xy
        self.kp_z = kp_z

    def compute_velocities(self, drone_z: float, target_x: float, target_y: float) -> dict:
        """Computes relative velocity setpoints based on target offset and altitude."""
        vx = self.kp_xy * target_x
        vy = self.kp_xy * target_y
        
        # Descend rate proportional to height, clamped between -1.0 m/s and -0.1 m/s
        vz = -max(0.1, min(1.0, self.kp_z * drone_z))

        return {'vx': vx, 'vy': vy, 'vz': vz}