"""Where the pad is in the Pixhawk's own position frame (EKF LOCAL_POSITION_NED), for GUIDED
position targets.

The camera measures the touchdown point relative to the drone (forward / right, metres). Adding
the drone's EKF position AT THE TIME OF THE FRAME (latest LOCAL_POSITION_NED moved back by its
velocity) turns it into a point that does not move when the drone moves: a still pad gives the
same north/east whatever the drone does, so the Pi's processing delay does not enter the
Pixhawk's position loop (the old GUIDED sent velocities and its delay made it oscillate).

The point is low-pass filtered (time constant tau_s). A measurement farther than jump_m from the
estimate is ignored unless jump_n of them in a row agree (the pad really moved / a new marker):
then the estimate restarts there."""
import math
from typing import Optional, Tuple

NE = Tuple[float, float]


def body_to_ne(fwd: float, right: float, yaw_rad: float) -> NE:
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    return fwd * c - right * s, fwd * s + right * c


class PadLocator:
    def __init__(self, tau_s: float = 0.4, jump_m: float = 0.25, jump_n: int = 3, max_back_s: float = 0.5):
        self.tau_s, self.jump_m, self.jump_n, self.max_back_s = tau_s, jump_m, jump_n, max_back_s
        self.reset()

    def reset(self):
        self.est: Optional[NE] = None
        self.t: Optional[float] = None      # time of the last accepted measurement
        self._jumps = 0

    @staticmethod
    def drone_at(pos_ne: NE, vel_ne: NE, back_s: float) -> NE:
        """Drone position back_s seconds before the position packet."""
        return pos_ne[0] - vel_ne[0] * back_s, pos_ne[1] - vel_ne[1] * back_s

    def update(self, fwd: float, right: float, yaw_rad: float, pos_ne: NE, vel_ne: NE,
               back_s: float, now: float) -> NE:
        """fwd/right: pad from the drone at the frame time; pos/vel: latest LOCAL_POSITION_NED;
        back_s: seconds from the frame to that packet (positive = frame older)."""
        back_s = max(-self.max_back_s, min(self.max_back_s, back_s))
        dn, de = self.drone_at(pos_ne, vel_ne, back_s)
        on, oe = body_to_ne(fwd, right, yaw_rad)
        meas = (dn + on, de + oe)
        if self.est is None:
            self.est, self.t, self._jumps = meas, now, 0
            return self.est
        if math.hypot(meas[0] - self.est[0], meas[1] - self.est[1]) > self.jump_m:
            self._jumps += 1
            if self._jumps < self.jump_n:
                return self.est
            self.est, self.t, self._jumps = meas, now, 0
            return self.est
        self._jumps = 0
        dt = max(0.0, now - (self.t if self.t is not None else now))
        k = 1.0 - math.exp(-dt / self.tau_s) if self.tau_s > 0 else 1.0
        self.est = (self.est[0] + k * (meas[0] - self.est[0]), self.est[1] + k * (meas[1] - self.est[1]))
        self.t = now
        return self.est

    def fresh(self, now: float, max_age_s: float) -> bool:
        return self.est is not None and self.t is not None and now - self.t <= max_age_s
