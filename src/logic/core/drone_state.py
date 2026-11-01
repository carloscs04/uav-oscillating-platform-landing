from dataclasses import dataclass

@dataclass
class DroneState:
    """Pure Python data container for UAV telemetry."""
    timestamp: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    armed: bool = False
    flight_mode: str = "UNKNOWN"
