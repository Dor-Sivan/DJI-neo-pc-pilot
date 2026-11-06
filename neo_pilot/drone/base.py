"""
Drone-agnostic interface: what a flight program sees of any drone.

Programs only use Frame, State and the Drone methods below. Each drone type
gets its own adapter (drone/neo.py for the DJI Neo through the phone, drone/sim.py
for a simulated Neo, drone/replay.py for a recording) that fills these in.
Switching drones = writing a new adapter, nothing else.

Conventions:
  times      PC clock, unix seconds (time.time()). Each adapter converts its
             drone's clock to this.
  world      local ENU, metres: x = east, y = north, z = up. Origin = home
             (where the adapter first had a position fix).
  angles     degrees. yaw = compass heading as the drone reports it
             (0 = north, clockwise). heading_enu() converts it to ENU
             (0 = east, counter-clockwise).
"""
import math
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class Frame:
    image: np.ndarray       # BGR, camera area only
    t: float                # capture time (estimate), PC clock
    t_arrival: float        # when the PC received it
    seq: int                # 0, 1, 2 ... since connect


@dataclass
class State:
    t: float                    # measurement time (estimate), PC clock
    t_arrival: float            # when the PC received it
    seq: int
    pos_enu: np.ndarray | None  # (3,) metres from home; None without a position fix
    vel_enu: np.ndarray         # (3,) m/s
    height: float               # metres above the takeoff point (taking off from a table: table = 0)
    roll: float
    pitch: float
    yaw: float                  # compass heading, 0 = north, clockwise
    gimbal_pitch: float
    gimbal_yaw: float
    lat: float | None           # degrees; None without GPS
    lon: float | None
    gps_ok: bool
    motors_on: bool
    mode: str                   # drone's own flight mode name
    battery: float              # percent
    raw: dict                   # everything the drone sent, for logging
    ground: float | None = None     # metres to the surface right below (downward sensor), None if not measured

    def heading_enu(self):
        return 90.0 - self.yaw


def inverse_map(table, want):
    """
    Stick for a wanted response from a (stick, response) table; same sign, clipped to the table's end.
    The first point is the dead-zone edge: a wanted response of half the edge's or less -> 0 (no stick),
    more -> at least the edge stick, so the response the code expects is one the drone really gives.
    """
    s, r = zip(*table)
    a = abs(want)
    if a <= r[0] / 2:
        return 0.0
    return math.copysign(float(np.interp(a, r, s)), want)


def forward_map(table, stick):
    """Expected response for a stick value: 0 inside the dead zone."""
    s, r = zip(*table)
    a = abs(stick)
    if a < s[0]:
        return 0.0
    return math.copysign(float(np.interp(a, s, r)), stick)


class Drone(ABC):
    """
    A drone as a program sees it: frames, states, and stick-like control.

    An adapter calls self._new_frame(frame) for every frame it receives; this
    class keeps the newest one and passes each to the registered listeners.

    Delays differ per drone and link, so they are part of the interface:
      video_delay        capture -> PC for frames, seconds. Set by the adapter
                         (measured once, or estimated live by overriding it).
      telemetry_delay()  measurement -> PC for states, seconds. Measured live
                         from the states received (median of the last ones).
    A consumer must not assume either is fixed: it waits until the states
    cover the time it needs (state_at(t)).

    Control, like a remote controller in normal (positioning) mode:
      sticks(yaw, up, right, forward)   raw sticks -1..1, clipped to max_stick; each
                         command holds only briefly (adapter's timeout), so call
                         it every step. release() lets go (the drone brakes, hovers).
      rates(yaw_rate, climb, right, forward)   deg/s and m/s (drone frame) through
                         the adapter's measured maps YAW_MAP, UP_MAP, SPEED_MAP.
      velocity_enu(east, north, up, yaw_rate)  world frame, turned into the drone's
                         frame with the newest heading.
    """

    video_delay = 0.0
    max_stick = 0.5
    YAW_MAP = UP_MAP = SPEED_MAP = None     # (stick, response) points, set by the adapter

    def __init__(self):
        self._frame = None
        self._listeners = []

    def telemetry_delay(self, last=50) -> float | None:
        s = self.states()[-last:]
        if not s:
            return None
        return float(np.median([x.t_arrival - x.t for x in s]))

    def _new_frame(self, frame: Frame):
        self._frame = frame
        for fn in self._listeners:
            fn(frame)

    def add_frame_listener(self, fn):
        """fn(frame) is called for every frame, on the video thread: keep it fast
        (e.g. put the frame on a queue)."""
        self._listeners.append(fn)

    def latest_frame(self) -> Frame | None:
        """Newest camera frame, or None before the first one."""
        return self._frame

    @abstractmethod
    def states(self) -> list[State]:
        """All telemetry states received since connect, oldest first."""

    @abstractmethod
    def alive(self) -> bool:
        """False once the video or telemetry link is gone."""

    @abstractmethod
    def close(self):
        ...

    def latest_state(self) -> State | None:
        s = self.states()
        return s[-1] if s else None

    def state_at(self, t) -> State | None:
        """
        State at time t, linearly interpolated between the two states around it
        (e.g. the attitude at a frame's capture time). None if t is outside the
        received range.
        """
        s = self.states()
        if not s or t < s[0].t or t > s[-1].t:
            return None
        k = int(np.searchsorted([x.t for x in s], t))
        if k == 0 or s[k].t == t:
            return s[k]
        a, b = s[k - 1], s[k]
        w = (t - a.t) / (b.t - a.t)
        lerp = lambda x, y: x + w * (y - x)
        ang = lambda x, y: (x + w * ((y - x + 180) % 360 - 180) + 180) % 360 - 180    # shortest way round
        return State(
            t=t, t_arrival=b.t_arrival, seq=b.seq,
            pos_enu=None if a.pos_enu is None or b.pos_enu is None else lerp(a.pos_enu, b.pos_enu),
            vel_enu=lerp(a.vel_enu, b.vel_enu), height=lerp(a.height, b.height),
            roll=lerp(a.roll, b.roll), pitch=lerp(a.pitch, b.pitch), yaw=ang(a.yaw, b.yaw),
            gimbal_pitch=lerp(a.gimbal_pitch, b.gimbal_pitch), gimbal_yaw=ang(a.gimbal_yaw, b.gimbal_yaw),
            lat=b.lat, lon=b.lon, gps_ok=b.gps_ok, motors_on=b.motors_on, mode=b.mode,
            battery=b.battery, raw=b.raw,
            ground=None if a.ground is None or b.ground is None else lerp(a.ground, b.ground))

    # ---- control ----
    def sticks(self, yaw=0.0, up=0.0, right=0.0, forward=0.0):
        raise NotImplementedError(f"{type(self).__name__} has no control")

    def release(self):
        raise NotImplementedError(f"{type(self).__name__} has no control")

    def command(self):
        """Stick values being applied now (yaw, up, right, forward)."""
        return (0.0, 0.0, 0.0, 0.0)

    def rate_sticks(self, yaw_rate=0.0, climb=0.0, right=0.0, forward=0.0):
        """Sticks for wanted rates (deg/s, m/s), through the measured maps."""
        return (inverse_map(self.YAW_MAP, yaw_rate), inverse_map(self.UP_MAP, climb),
                inverse_map(self.SPEED_MAP, right), inverse_map(self.SPEED_MAP, forward))

    def rates(self, yaw_rate=0.0, climb=0.0, right=0.0, forward=0.0):
        """Turn (deg/s, + = right), climb (m/s), move right/forward (m/s, drone frame). Speeds are reached after
        the drone accelerates (~1-2 s)."""
        self.sticks(*self.rate_sticks(yaw_rate, climb, right, forward))

    def expected_rates(self, cmd=None):
        """(yaw_rate, climb, right, forward) the sticks cmd (default: the current ones) should give."""
        yaw, up, right, forward = self.command() if cmd is None else cmd
        return (forward_map(self.YAW_MAP, yaw), forward_map(self.UP_MAP, up),
                forward_map(self.SPEED_MAP, right), forward_map(self.SPEED_MAP, forward))

    def velocity_enu(self, east=0.0, north=0.0, up=0.0, yaw_rate=0.0):
        """Move at (east, north) m/s in the world, climb at up m/s, turn at yaw_rate deg/s."""
        s = self.latest_state()
        y = math.radians(s.yaw if s is not None else 0.0)
        forward = east * math.sin(y) + north * math.cos(y)
        right = east * math.cos(y) - north * math.sin(y)
        self.rates(yaw_rate=yaw_rate, climb=up, right=right, forward=forward)

    def wait_ready(self, timeout=30.0):
        """Block until there is at least one frame and one state."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.latest_frame() is not None and self.latest_state() is not None:
                return True
            time.sleep(0.1)
        return False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
