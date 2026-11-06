"""
Safety layer between the flight tests and the drone.

SafeDrone wraps any Drone: every command goes through the profile's limits
(stick cap, rate caps, height band), and it remembers what it ordered.
Monitor checks, every loop while a test runs, for reasons to stop:
  - telemetry or video stopped arriving (stale)
  - the drone left the fence around where the test started
  - the sticks get no answer (a DJI pop-up takes the touches; seen 2026-10-31
    before an auto-landing: yaw and up were sent for 2 s, nothing moved)
"""
import math
import time
from collections import deque

import numpy as np

ACT_DELAY = 0.3             # s from a stick change to the drone answering (stick tests, rough)
TEL_STALE = 2.0             # s without a new telemetry row (rows come in batches every ~0.7 s)
TEL_LATE = 3.0              # s; rows still come but this old (normal ~1.5 s; 4 Nov route B: grew to 8.5 s and
                            # the follower swung the yaw full left / full right on the old data)
VIDEO_STALE = 1.0           # s without a new frame
CMD_LASTS = 0.3             # s a stick command holds (the adapters drop it after this)


def body_velocity(s):
    """(forward, right) m/s of state s, from its ENU velocity and heading."""
    y = math.radians(s.yaw)
    e, n = s.vel_enu[0], s.vel_enu[1]
    return e * math.sin(y) + n * math.cos(y), e * math.cos(y) - n * math.sin(y)


def low_height(s):
    """Height above the surface right below: the ground sensor if it reads, else above the takeoff point (taking
    off from a table makes that too low: it would block every descent)."""
    return s.height if s.ground is None else s.ground


class SafeDrone:
    """Same control calls as a Drone, with the profile's limits; everything else passes through."""

    def __init__(self, drone, profile):
        self.drone = drone
        self.profile = profile
        drone.max_stick = profile.max_stick
        self.history = deque(maxlen=2000)       # (time, sticks as sent)

    def __getattr__(self, name):
        return getattr(self.drone, name)

    def sticks(self, yaw=0.0, up=0.0, right=0.0, forward=0.0):
        s = self.drone.latest_state()
        if s is not None:
            if up > 0 and s.height >= self.profile.h_max:
                up = 0.0
            if up < 0 and low_height(s) <= self.profile.h_min:
                up = 0.0
        self.drone.sticks(yaw=yaw, up=up, right=right, forward=forward)
        self.history.append((time.time(), self.drone.command()))

    def release(self):
        self.drone.release()
        self.history.append((time.time(), (0.0, 0.0, 0.0, 0.0)))

    def rates(self, yaw_rate=0.0, climb=0.0, right=0.0, forward=0.0):
        p = self.profile
        yaw_rate = float(np.clip(yaw_rate, -p.max_yaw_rate, p.max_yaw_rate))
        climb = float(np.clip(climb, -p.max_climb, p.max_climb))
        speed = math.hypot(right, forward)
        if speed > p.max_speed:
            right, forward = right * p.max_speed / speed, forward * p.max_speed / speed
        self.sticks(*self.drone.rate_sticks(yaw_rate, climb, right, forward))

    def velocity_enu(self, east=0.0, north=0.0, up=0.0, yaw_rate=0.0):
        s = self.drone.latest_state()
        y = math.radians(s.yaw if s is not None else 0.0)
        self.rates(yaw_rate=yaw_rate, climb=up, right=east * math.cos(y) - north * math.sin(y),
                   forward=east * math.sin(y) + north * math.cos(y))


class Monitor:
    """Reasons to stop a running test. check() returns None or the reason."""

    def __init__(self, safe):
        self.safe = safe
        self.start_pos = None           # GPS position at the start (outdoors)
        self.t_start = None

    def start(self):
        d = self.safe.drone
        s = d.latest_state()
        self.t_start = time.time()
        self.start_pos = s.pos_enu.copy() if s is not None and s.pos_enu is not None else None

    def distance(self):
        """Metres from where the test started: GPS if both ends have it, else the integrated velocity."""
        d = self.safe.drone
        s = d.latest_state()
        if self.start_pos is not None and s is not None and s.pos_enu is not None:
            return float(np.linalg.norm(s.pos_enu[:2] - self.start_pos[:2])), "GPS"
        states = [x for x in d.states() if x.t > self.t_start]
        e = sum(x.vel_enu[0] for x in states) * 0.1
        n = sum(x.vel_enu[1] for x in states) * 0.1
        return math.hypot(e, n), "velocity"

    def check(self, now):
        d = self.safe.drone
        s = d.latest_state()
        if s is None or now - s.t_arrival > TEL_STALE:
            return "telemetry stopped"
        if now - s.t > TEL_LATE:
            return f"telemetry {now - s.t:.1f} s late"
        f = d.latest_frame()
        if f is None or now - f.t_arrival > VIDEO_STALE:
            return "video stopped"
        dist, how = self.distance()
        if dist > self.safe.profile.fence:
            return f"{dist:.1f} m from the start ({how}), fence {self.safe.profile.fence} m"
        return self.no_answer(d)

    def no_answer(self, d):
        """Sticks held steadily (by the expected response) but the drone does not move that way."""
        states = d.states()
        if len(states) < 12:
            return None
        T = states[-1].t
        recent = [x for x in states if x.t > T - 0.6]
        hist = list(self.safe.history)

        def steady(axis, hold, threshold):
            """Smallest |expected response| over the commands that drove the recent states (same sign), or 0.
            The command in effect at the window's start counts too, and a gap in the log longer than a command
            lasts means the sticks were released then."""
            a, b = T - hold - ACT_DELAY, T - ACT_DELAY
            before = [h for h in hist if h[0] < a]
            inside = [h for h in hist if a <= h[0] <= b]
            if not before or len(inside) < 5:
                return 0.0
            times = [before[-1][0]] + [t for t, _ in inside] + [b]
            if max(t2 - t1 for t1, t2 in zip(times, times[1:])) > CMD_LASTS:
                return 0.0
            win = [d.expected_rates(c)[axis] for _, c in [before[-1]] + inside]
            lo, hi = min(win), max(win)
            if lo >= threshold:
                return lo
            if hi <= -threshold:
                return hi
            return 0.0

        # yaw from 15 deg/s only: smaller turns sit at the stick's dead-zone edge, which gives far less outdoors than
        # the map (4 Nov: 12 ordered, 2-4 deg/s measured in forward flight: a false abort)
        want = steady(0, 1.0, 15.0)
        def yaw_rate(win):
            if len(win) < 3:
                return None
            yaw = np.degrees(np.unwrap(np.radians([x.yaw for x in win])))
            return (yaw[-1] - yaw[0]) / (win[-1].t - win[0].t)

        rate = yaw_rate(recent)
        if want and rate is not None:
            # a turn still going the other way (the drone swinging in strong wind, 4 Nov 11:59: +18 ordered, -19
            # measured while DJI's record shows it answering) counts as an answer while the rate moves towards the order
            was = yaw_rate([x for x in states if T - 1.2 < x.t <= T - 0.6])
            gap = want - (was if was is not None else 0.0)
            if rate * want < 0.25 * want * want and (rate - want + gap) * gap < 0.25 * gap * gap:
                return f"sticks get no answer: yaw {want:+.0f} deg/s ordered, {rate:+.0f} measured (pop-up?)"
        for axis, name, k in ((3, "forward", 0), (2, "right", 1)):
            want = steady(axis, 2.0, 0.15)
            if want and recent:
                got = float(np.mean([body_velocity(x)[k] for x in recent]))
                # a reversal from full speed first brakes for ~2 s: that counts as an answer while the speed moves
                # towards the order (4 Nov: -1.3 ordered, -0.2 measured after braking from +1.96: a false abort)
                early = [body_velocity(x)[k] for x in states if T - 2.6 <= x.t <= T - 2.0]
                was = float(np.mean(early)) if early else 0.0
                gap = want - was
                if got * want < 0.25 * want * want and (got - was) * gap < 0.25 * gap * gap:
                    return f"sticks get no answer: {name} {want:+.1f} m/s ordered, {got:+.1f} measured (pop-up?)"
        return None
