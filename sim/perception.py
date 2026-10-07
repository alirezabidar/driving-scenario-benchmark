"""'wide' sensing: watch the kerbs as well as the lane, and yield to people stepping out.

Plain Python so the same code runs in the dry run and in Isaac Sim. Input each step
is a list of side-ray hits (forward_m, lateral_m) in the car frame, measured from the
front bumper. Output is an 'effective obstacle distance' for RouteController.update,
which already caps speed at sqrt(2 * 1.0 * (d - 3)).
"""
import math

FAN_ANGLES_DEG = (-60, -45, -30, -15, 15, 30, 45, 60)
FAN_RANGE_M = 20.0
WATCH_LATERAL_M = 6.0        # ignore anything further to the side (buildings, far kerb)
CAUTION_LATERAL_M = 4.0      # someone this close to the lane slows the car
CAUTION_SPEED_MPS = 1.2
CAR_HALF_WIDTH_M = 1.025
CAR_LENGTH_M = 4.9
STOP_MARGIN_M = 3.0
DECEL = 1.0


def _distance_for_speed(v):
    """Obstacle distance at which the controller's speed cap equals v."""
    return STOP_MARGIN_M + v * v / (2 * DECEL)


class SideWatch:
    def __init__(self, dt):
        self.dt = dt
        self.lat = None          # |lateral| of the nearest watched object
        self.rate = 0.0          # d|lateral|/dt, negative = moving towards the lane
        self.age = 0.0           # seconds since the last hit
        self.state = 'clear'
        self.fwd = None
        self._last = math.inf

    def update(self, hits, speed):
        near = [(f, l) for f, l in hits if abs(l) < WATCH_LATERAL_M and f > 0]
        if near:
            f, l = min(near, key=lambda h: abs(h[1]))
            if self.lat is not None and self.age < 0.5:
                raw = (abs(l) - self.lat) / max(self.dt, self.age + self.dt)
                self.rate = 0.8 * self.rate + 0.2 * raw
            self.lat, self.fwd, self.age = abs(l), f, 0.0
        else:
            self.age += self.dt
            if self.age > 0.5:
                self.lat, self.rate, self.state = None, 0.0, 'clear'
                return math.inf
            return self._last
        # Constant-velocity prediction: will they reach the car's side of the lane
        # before the car has driven past them?
        gap = self.lat - (CAR_HALF_WIDTH_M + 0.5)
        if self.rate < -0.3 and gap > 0:
            t_reach = gap / -self.rate
            t_pass = (self.fwd + CAR_LENGTH_M + 1.0) / max(speed, 0.5)
            if t_reach < t_pass + 1.0:
                self.state = 'yield'
                self._last = max(self.fwd, STOP_MARGIN_M)
                return self._last
        if gap <= 0:
            self.state = 'yield'          # already in or at the edge of the lane
            self._last = max(self.fwd, STOP_MARGIN_M)
            return self._last
        if self.lat < CAUTION_LATERAL_M:
            self.state = 'caution'
            self._last = max(self.fwd, _distance_for_speed(CAUTION_SPEED_MPS))
            return self._last
        self.state = 'watch'
        self._last = math.inf
        return self._last
