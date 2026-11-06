"""
The flight tests. Each one has a key, the environments it is allowed in, and:
  start(run)        called when its key is pressed (checks passed)
  step(now)         every loop; returns False when finished
  on_frame(f, now)  every new video frame (vision tests)
  overlay()         lines for the window
  report(states)    lines printed after the run (the late telemetry has arrived)

Commands go through ctx.safe (flight/safety.py: SafeDrone with the profile's
limits). Telemetry is ~0.8 s late, frames ~0.4 s: reports run after the end.
"""
import csv
import math
import os
import sys
import time

import cv2
import numpy as np

from .safety import ACT_DELAY, body_velocity


def unwrap_deg(a):
    return np.degrees(np.unwrap(np.radians(a)))


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def window(states, a, b):
    return [s for s in states if a <= s.t <= b]


def mean_response(states, a, b):
    """Mean yaw rate (deg/s), climb (m/s), forward and right speed (m/s) of the states in [a, b]."""
    w = window(states, a, b)
    if len(w) < 3:
        return None
    yaw = unwrap_deg([s.yaw for s in w])
    rate = (yaw[-1] - yaw[0]) / (w[-1].t - w[0].t)
    fr = np.array([body_velocity(s) for s in w])
    return rate, float(np.mean([s.vel_enu[2] for s in w])), float(fr[:, 1].mean()), float(fr[:, 0].mean())


class Test:
    key = ""
    name = ""
    title = ""
    envs = ("indoor", "outdoor")
    needs_colour = False
    sim_ok = True

    def __init__(self, ctx):
        self.ctx = ctx
        self.run = None
        self.t0 = None

    @property
    def safe(self):
        return self.ctx.safe

    def can_start(self):
        """None if it may start now, else why not."""
        if self.ctx.profile.name not in self.envs:
            return f"{self.name} is an {'/'.join(self.envs)} test"
        if self.ctx.sim and not self.sim_ok:
            return f"{self.name} does not run in the simulator"
        if self.needs_colour and self.ctx.detector.hsv is None:
            return "click on the coloured target first"
        return None

    def start(self, run):
        self.run = run
        self.t0 = time.time()

    def step(self, now):
        return False

    def on_frame(self, f, now):
        pass

    def overlay(self):
        return []

    def marker(self):
        return None

    def report(self, states):
        return []


# ---------------- safety ----------------
class Kill(Test):
    key, name = "1", "kill"
    title = "KILL: hold yaw 0.4 for 1 s, then the program dies (watchdog must release). Ends the program"
    sim_ok = False

    def step(self, now):
        self.safe.sticks(yaw=0.4)
        self.run.log_cmd(self.safe.command(), now)
        if now - self.t0 >= 1.0:
            self.run.t_end = now
            print("\nKILL: yaw 0.4 held, the program dies now without releasing. Watch the drone.")
            self.run.save(self.ctx.drone.states())
            sys.stdout.flush()
            os._exit(1)
        return True


class Hang(Test):
    key, name = "h", "hang"
    title = "HANG: hold yaw 0.4 for 1 s, then the whole program freezes 3 s (watchdog must release after ~1 s)"

    def start(self, run):
        super().start(run)
        self.t_freeze = None

    def step(self, now):
        if now - self.t0 < 1.0:
            self.safe.sticks(yaw=0.4)
            self.run.log_cmd(self.safe.command(), now)
            return True
        self.t_freeze = time.time()
        self.run.event("freezing 3 s with yaw 0.4 held")
        self.ctx.drone.freeze_for_test(3.0)
        self.safe.release()
        self.run.log_cmd(self.safe.command(), time.time())
        return False

    def report(self, states):
        if self.t_freeze is None:
            return ["hang: did not reach the freeze"]
        w = window(states, self.t_freeze - 1.0, self.t_freeze + 4.0)
        if len(w) < 5:
            return ["hang: no telemetry around the freeze"]
        t = np.array([s.t for s in w])
        rate = np.gradient(unwrap_deg([s.yaw for s in w]), t)
        turning = rate[t < self.t_freeze].mean() if (t < self.t_freeze).any() else float("nan")
        after = np.flatnonzero((t > self.t_freeze) & (np.abs(rate) < 3.0))
        stop = f"stopped {t[after[0]] - self.t_freeze:.1f} s after the freeze" if len(after) else "DID NOT STOP"
        return [f"hang: turning {turning:+.0f} deg/s before the freeze; {stop} "
                "(expected ~1.3 s: watchdog 1 s + drone ~0.3 s)"]


# ---------------- pulse tests ----------------
class Pulses(Test):
    """A fixed sequence of stick pulses: (sticks (yaw, up, right, forward), seconds on, seconds released after)."""
    PULSES = []

    def start(self, run):
        super().start(run)
        self.schedule = []          # (t_on, t_off, sticks)
        t = self.t0
        for cmd, on, pause in self.PULSES:
            self.schedule.append((t, t + on, cmd))
            t += on + pause
        self.t_end = t

    def step(self, now):
        on = [p for p in self.schedule if p[0] <= now < p[1]]
        if on:
            self.safe.sticks(*on[0][2])
        else:
            self.safe.release()
        self.run.log_cmd(self.safe.command(), now)
        k = sum(1 for p in self.schedule if p[0] <= now)
        self.progress = f"pulse {k}/{len(self.schedule)}"
        return now < self.t_end

    def overlay(self):
        return [getattr(self, "progress", "")]


class Combined(Pulses):
    key, name = "c", "combined"
    title = "TWO FINGERS: both sticks at once (forward+yaw, right+up, all four), each then reversed"
    envs = ("outdoor",)
    PULSES = [((0.3, 0, 0, 0.6), 3, 2), ((-0.3, 0, 0, -0.6), 3, 2),
              ((0, 0.6, 0.6, 0), 3, 2), ((0, -0.6, -0.6, 0), 3, 2),
              ((0.3, 0.6, 0.6, 0.6), 3, 2), ((-0.3, -0.6, -0.6, -0.6), 3, 2)]

    def report(self, states):
        lines = ["two fingers (second half of each pulse; expected from the single-axis maps):",
                 "  sticks (yaw, up, right, fwd)    yaw deg/s     climb m/s    right m/s    fwd m/s   (got / expected)"]
        for t_on, t_off, cmd in self.schedule:
            m = mean_response(states, (t_on + t_off) / 2 + ACT_DELAY, t_off + ACT_DELAY)
            e = self.safe.expected_rates(cmd)
            if m is None:
                lines.append(f"  {cmd}: no telemetry")
                continue
            lines.append("  ({:+.2f} {:+.2f} {:+.2f} {:+.2f})  ".format(*cmd) + "  ".join(
                f"{g:+6.2f}/{x:+6.2f}" for g, x in zip(m, e)))
        return lines


class SpeedTable(Pulses):
    """Full-speed table along one axis, each value forward then back (wind cancels in the average)."""
    envs = ("outdoor",)
    AXIS = 3            # 3 = forward, 2 = right (index in the sticks tuple)
    VALUES = (0.5, 0.7, 1.0)
    ON = 3.0
    PAUSE = staticmethod(lambda v: 2.0 + 4.0 * v)   # s released after each pulse: braking from top speed takes longer

    def start(self, run):
        self.PULSES = []
        for v in self.VALUES:
            for sign in (+1, -1):
                cmd = [0.0, 0.0, 0.0, 0.0]
                cmd[self.AXIS] = sign * v
                self.PULSES.append((tuple(cmd), self.ON, self.PAUSE(v)))
        super().start(run)

    def report(self, states):
        k = 0 if self.AXIS == 3 else 1          # body_velocity -> (forward, right)
        axis = "forward" if self.AXIS == 3 else "right"
        lines = [f"speed table, {axis} (body frame; brake = after release until < 0.3 m/s):",
                 "  stick   steady m/s  peak m/s  to 90% s  brake s  brake m"]
        result = {}
        for t_on, t_off, cmd in self.schedule:
            v = cmd[self.AXIS]
            w = window(states, t_on - 0.2, t_off + self.PAUSE(abs(v)))
            if len(w) < 10:
                lines.append(f"  {v:+.1f}  no telemetry")
                continue
            t = np.array([s.t for s in w])
            sp = np.array([body_velocity(s)[k] for s in w])
            end = t_off + ACT_DELAY
            steady = sp[(t >= end - 0.5) & (t <= end)].mean()
            peak = sp[np.argmax(sp * np.sign(v))]          # largest speed in the pulse's direction
            reach = np.flatnonzero((t >= t_on) & (np.abs(sp) >= 0.9 * abs(steady)))
            t90 = t[reach[0]] - t_on if len(reach) else float("nan")
            stop = np.flatnonzero((t > end) & (np.abs(sp) < 0.3))
            if len(stop):
                b = (t > end) & (t <= t[stop[0]])
                brake_t, brake_d = t[stop[0]] - t_off, abs(np.sum(sp[b]) * 0.1)
            else:
                brake_t = brake_d = float("nan")
            result[v] = steady
            lines.append(f"  {v:+.2f}  {steady:+7.2f}    {peak:+7.2f}   {t90:6.1f}   {brake_t:6.1f}  {brake_d:6.1f}")
        for v in self.VALUES:
            if v in result and -v in result:
                a, b = abs(result[v]), abs(result[-v])
                lines.append(f"  stick {v:.2f}: {(a + b) / 2:.2f} m/s average (wind changes it by +-{abs(a - b) / 2:.2f})")
        return lines


class SpeedForward(SpeedTable):
    key, name = "s", "speed_fwd"
    title = "FULL SPEED forward/back: sticks 0.5, 0.7, 1.0 for 3 s each way. Needs ~40 m clear ahead and behind"
    AXIS = 3


class SpeedRight(SpeedTable):
    key, name = "d", "speed_right"
    title = "FULL SPEED right/left: sticks 0.5, 0.7, 1.0 for 3 s each way. Needs ~40 m clear to both sides"
    AXIS = 2


class SlowForward(SpeedTable):
    """Approach found stick 0.3 (the map's dead-zone edge, "0.2 m/s") gave 0 m/s indoors: the slow end, measured
    where the approach uses it."""
    key, name = "w", "slow_fwd"
    title = "SLOW forward/back: sticks 0.3, 0.35, 0.4, 0.45, 0.5 for 2 s each way (~1 m). Needs ~1.5 m clear ahead/behind"
    envs = ("indoor",)
    AXIS = 3
    VALUES = (0.3, 0.35, 0.4, 0.45, 0.5)
    ON = 2.0
    PAUSE = staticmethod(lambda v: 2.0)


class YawUp(Pulses):
    key, name = "v", "yaw_up_max"
    title = "MAX YAW and CLIMB: yaw 0.6, 1.0 (1.5 s each way), up 0.6, 1.0 (2 s up, then 2 s down)"
    envs = ("outdoor",)
    PULSES = [((0.6, 0, 0, 0), 1.5, 1.5), ((-0.6, 0, 0, 0), 1.5, 1.5),
              ((1.0, 0, 0, 0), 1.5, 1.5), ((-1.0, 0, 0, 0), 1.5, 1.5),
              ((0, 0.6, 0, 0), 2, 1.5), ((0, -0.6, 0, 0), 2, 1.5),
              ((0, 1.0, 0, 0), 2, 1.5), ((0, -1.0, 0, 0), 2, 1.5)]

    def report(self, states):
        lines = ["max yaw / climb (second half of each pulse):"]
        for t_on, t_off, cmd in self.schedule:
            m = mean_response(states, (t_on + t_off) / 2 + ACT_DELAY, t_off + ACT_DELAY)
            if m is None:
                lines.append(f"  {cmd}: no telemetry")
            elif cmd[0]:
                lines.append(f"  yaw {cmd[0]:+.1f}: {m[0]:+6.1f} deg/s")
            else:
                lines.append(f"  up  {cmd[1]:+.1f}: {m[1]:+5.2f} m/s")
        return lines


class Turns(Pulses):
    key, name = "o", "turns"
    title = "TURNS for the frame/telemetry offset: yaw +-0.8, 1 s each, 8 times. Point at a textured scene"
    PULSES = [((0.8 * (-1) ** k, 0, 0, 0), 1.0, 0.3) for k in range(8)]
    WIDTH = 320

    def start(self, run):
        super().start(run)
        self.frames = []            # (t_frame, small grey image)

    def on_frame(self, f, now):
        h, w = f.image.shape[:2]
        small = cv2.resize(cv2.cvtColor(f.image, cv2.COLOR_BGR2GRAY), (self.WIDTH, int(h * self.WIDTH / w)))
        self.frames.append((f.t, small.astype(np.float32), w))

    def report(self, states):
        if len(self.frames) < 20:
            return ["turns: too few frames"]
        win = np.hanning(self.frames[0][1].shape[1])[None, :] * np.hanning(self.frames[0][1].shape[0])[:, None]
        tm, rate_img = [], []
        for (t1, a, _), (t2, b, _) in zip(self.frames, self.frames[1:]):
            if t2 - t1 <= 0:
                continue
            (dx, _), _ = cv2.phaseCorrelate(a, b, win.astype(np.float32))
            tm.append((t1 + t2) / 2)
            rate_img.append(dx / (t2 - t1))
        tm, rate_img = np.array(tm), np.array(rate_img)
        w = window(states, tm[0] - 2, tm[-1] + 2)
        if len(w) < 10:
            return ["turns: no telemetry"]
        ts = np.array([s.t for s in w])
        yr = np.gradient(unwrap_deg([s.yaw for s in w]), ts)
        best, best_c = 0.0, -2.0
        for tau in np.arange(-0.6, 1.0, 0.01):
            y = np.interp(tm - tau, ts, yr)
            c = np.corrcoef(-rate_img, y)[0, 1]
            if c > best_c:
                best, best_c = tau, c
        y = np.interp(tm - best, ts, yr)
        slope = float(np.dot(-rate_img, y) / np.dot(y, y))          # small-image px per degree
        full_w = self.frames[0][2]
        fx = slope * full_w / self.WIDTH * 180 / math.pi
        vd = self.ctx.drone.video_delay
        return [f"turns: {len(self.frames)} frames; image motion matches the yaw rate best when the frames are "
                f"{best:+.2f} s older than their time stamps (correlation {best_c:.2f})",
                f"  -> video_delay {vd:.3f} s should be {vd + best:.3f} s (frame vs telemetry, all delays together)",
                f"  image scale {slope * full_w / self.WIDTH:.1f} px/deg on the full frame -> fx ~ {fx:.0f} px"]


class Landmark(Test):
    key, name = "l", "landmark"
    title = "LANDMARK: put a far landmark of known bearing (--landmark DEG) under the centre cross, press l, hold 4 s"
    envs = ("outdoor",)

    def can_start(self):
        if self.ctx.args.landmark is None:
            return "start the program with --landmark <true compass bearing of the landmark>"
        return super().can_start()

    def step(self, now):
        self.safe.release()
        self.run.log_cmd(self.safe.command(), now)
        return now - self.t0 < 4.0

    def report(self, states):
        w = window(states, self.t0 + 0.5, self.t0 + 4.0)
        if len(w) < 5:
            return ["landmark: no telemetry"]
        L = self.ctx.args.landmark
        dy = float(np.mean(wrap180(np.array([s.yaw for s in w]) - L)))
        gy = float(np.mean(wrap180(np.array([s.gimbal_yaw for s in w]) - L)))
        return [f"landmark at {L:.1f} deg: drone yaw is {dy:+.1f} deg off, gimbal yaw {gy:+.1f} deg off "
                "(the smaller one is the better camera heading; a constant offset of a few degrees = magnetic "
                "declination if the bearing was true north)"]


# ---------------- square ----------------
class Square(Test):
    key, name = "2", "square"
    title = "SQUARE in the world: east, north, west, south (profile side and speed), 2 s hover between legs"
    LEGS = [("east", (1, 0)), ("north", (0, 1)), ("west", (-1, 0)), ("south", (0, -1))]
    BRAKE = 0.4         # s; braking distance ~ speed x this
    K_CROSS = 0.8       # 1/s; sideways correction per metre off the line
    PAUSE = 2.0

    def start(self, run):
        super().start(run)
        p = self.ctx.profile
        self.side, self.speed = p.square_side, p.square_speed
        self.timeout = 2 * self.side / self.speed + 3
        self.leg, self.moving, self.t_leg = 0, True, self.t0
        self.legs = []
        s = self.ctx.drone.latest_state()
        self.gps0 = s.pos_enu.copy() if s.pos_enu is not None else None

    def step(self, now):
        name, (de, dn) = self.LEGS[self.leg]
        direction = np.array([de, dn], float)
        states = self.ctx.drone.states()
        if self.moving:
            after = [s for s in states if s.t > self.t_leg]
            done = sum(float(s.vel_enu[:2] @ direction) * 0.1 for s in after)
            ahead = float(after[-1].vel_enu[:2] @ direction) * (now - after[-1].t + ACT_DELAY + self.BRAKE) \
                if after else 0.0
            why = "reached" if done + ahead >= self.side else "timeout" if now - self.t_leg > self.timeout else None
            if why is None:
                # steer back onto the leg's line: sideways drift since the leg started, corrected at K_CROSS per s
                across = np.array([-dn, de], float)
                off = sum(float(s.vel_enu[:2] @ across) * 0.1 for s in after)
                back = float(np.clip(-self.K_CROSS * off, -0.3, 0.3))
                self.safe.velocity_enu(east=de * self.speed + back * across[0], north=dn * self.speed + back * across[1])
            else:
                self.safe.release()
                self.legs.append((name, direction, self.t_leg, now, why))
                self.moving, self.t_leg = False, now
        else:
            self.safe.release()
            if now - self.t_leg >= self.PAUSE:
                self.leg += 1
                if self.leg == len(self.LEGS):
                    return False
                self.moving, self.t_leg = True, now
        self.run.log_cmd(self.safe.command(), now)
        return True

    def overlay(self):
        return [f"leg {self.leg + 1}/4 {self.LEGS[min(self.leg, 3)][0]}{'' if self.moving else ' (pause)'}"]

    def report(self, states):
        lines = [f"square {self.side} m at {self.speed} m/s (distances from the integrated velocity):"]
        for i, (name, direction, t0, t1, why) in enumerate(self.legs):
            nxt = self.legs[i + 1][2] if i + 1 < len(self.legs) else t1 + self.PAUSE
            seg = window(states, t0, nxt)
            across = np.array([-direction[1], direction[0]])
            along = sum(float(s.vel_enu[:2] @ direction) * 0.1 for s in seg)
            side = sum(float(s.vel_enu[:2] @ across) * 0.1 for s in seg)
            lines.append(f"  {name:5s} ({why:7s}): {along:+.2f} m along, {side:+.2f} m sideways")
        if self.legs:
            seg = window(states, self.legs[0][2], self.legs[-1][3] + self.PAUSE)
            e = sum(s.vel_enu[0] for s in seg) * 0.1
            n = sum(s.vel_enu[1] for s in seg) * 0.1
            lines.append(f"  end vs start: {math.hypot(e, n):.2f} m by velocity")
            end = window(states, self.legs[-1][3] + self.PAUSE - 0.5, self.legs[-1][3] + self.PAUSE + 0.5)
            if self.gps0 is not None and end and end[-1].pos_enu is not None:
                lines.append(f"  end vs start: {np.linalg.norm(end[-1].pos_enu[:2] - self.gps0[:2]):.2f} m by GPS")
        return lines


# ---------------- vision: follow and approach ----------------
class Tracker:
    """
    Target direction from the frames, corrected for the delays:
      ang_h, ang_v   angles off the image centre (nominal pinhole, focal length --fx px at 1920 px wide,
                     no distortion: pixel -> unit ray in the camera)
      pred_h         ang_h minus the turn already ordered since the frame was taken (Smith predictor)
      omega          the target's own turn rate (deg/s) seen from the drone: its world azimuth
                     (drone heading at the frame time + ang_h) through an alpha-beta filter. The
                     heading comes from the telemetry where it has arrived, else the last telemetry
                     heading plus the turn ordered since (the model).
      range          from the target's size: focal length x real diameter / diameter in pixels
    """
    K_YAW = 1.5         # deg/s per degree off-centre
    FF_GAIN = 0.8       # share of the target's own turn rate added on top
    K_CLIMB = 0.01      # m/s per degree off-centre
    DEADBAND = 2.0      # deg
    ALPHA, BETA = 0.25, 0.03

    def __init__(self, ctx):
        self.ctx = ctx
        self.ordered = []           # (time, expected yaw rate) of the sticks sent
        self.reset()

    def reset(self):
        self.az, self.omega, self.n, self.t_last = None, 0.0, 0, None

    def note_command(self, now):
        self.ordered.append((now, self.ctx.safe.expected_rates()[0]))
        self.ordered = [o for o in self.ordered if o[0] > now - 6.0]

    def model_turn(self, a, b):
        """Degrees turned between times a and b by the ordered rates (each answered ACT_DELAY later)."""
        total = 0.0
        for (t0, r), (t1, _) in zip(self.ordered, self.ordered[1:] + [(math.inf, 0.0)]):
            lo, hi = max(t0, a - ACT_DELAY), min(t1, b - ACT_DELAY)
            if hi > lo:
                total += r * (hi - lo)
        return total

    def heading_at(self, t):
        d = self.ctx.drone
        st = d.latest_state()
        if st is None:
            return None
        if t <= st.t:
            s = d.state_at(t)
            return (s or st).yaw
        return st.yaw + self.model_turn(st.t, t)

    def measure(self, f, now):
        det = self.ctx.detector(f.image)
        if det is None:
            return None
        h, w = f.image.shape[:2]
        fx = self.ctx.args.fx * w / 1920
        r = np.array([1.0, (det.u - w / 2) / fx, (det.v - h / 2) / fx])
        r /= np.linalg.norm(r)
        m = {"det": det, "f": f, "ray": r,
             "ang_h": math.degrees(math.atan2(r[1], r[0])), "ang_v": math.degrees(math.atan2(r[2], r[0]))}
        m["pred_h"] = m["ang_h"] - self.model_turn(f.t, now + ACT_DELAY)
        heading = self.heading_at(f.t)
        m["heading"] = heading
        if heading is not None:
            az = heading + m["ang_h"]
            if self.az is None:
                self.az, self.omega, self.n = az, 0.0, 1
            else:
                dt = max(f.t - self.t_last, 1e-3)
                pred = self.az + self.omega * dt
                resid = wrap180(az - pred)
                self.az = pred + self.ALPHA * resid
                self.omega = float(np.clip(self.omega + self.BETA / dt * resid, -60, 60))
                self.n += 1
            self.t_last = f.t
            m["az"], m["az_f"] = az, self.az
        m["omega"] = self.omega if self.n >= 10 else 0.0
        m["range"] = None
        if det.box:
            size = (det.box[2] + det.box[3]) / 2
            if size > 3:
                m["range"] = fx * self.ctx.args.ball_size / size / r[0]
        return m

    def yaw_rate(self, m):
        fb = 0.0 if abs(m["pred_h"]) < self.DEADBAND else self.K_YAW * m["pred_h"]
        return fb + self.FF_GAIN * m["omega"]

    def climb(self, m):
        return 0.0 if abs(m["ang_v"]) < self.DEADBAND else -self.K_CLIMB * m["ang_v"]


TRACK_COLUMNS = ["t_frame", "t_arrival", "seq", "u", "v", "size_px", "rF", "rR", "rD", "ang_h", "ang_v", "pred_h",
                 "heading", "az", "az_filtered", "omega", "range", "yaw_rate_cmd", "climb_cmd", "fwd_cmd",
                 "yaw_stick", "up_stick", "fwd_stick"]


class Follow(Test):
    key, name = "3", "follow"
    title = "FOLLOW the coloured target with yaw + height (predictor + target's own turn rate). 3 again = stop"
    LOST_AFTER = 0.5
    MAX_TIME = 60.0
    WAIT_CLICK = 30.0   # s hovering for the click on the target before giving up

    def start(self, run):
        super().start(run)
        self.tracker = Tracker(self.ctx)
        self.t_seen = self.t0
        # hover until the target is clicked (fresh colour every run); the simulator's ball colour is preset
        self.waiting = not self.ctx.sim
        if self.waiting:
            run.event("hovering: click the target to start")
        self.cmd = (0.0, 0.0, 0.0)          # yaw rate, climb, forward speed
        self.m = None
        self.rows = run.table("track", TRACK_COLUMNS)

    def forward(self, m, now):
        return 0.0

    def clicked(self, now):
        """The target was clicked (the detector has its colour): start following from now."""
        self.waiting = False
        self.t0 = self.t_seen = now
        self.run.event("target clicked: start")

    def on_frame(self, f, now):
        if self.waiting:
            return
        m = self.tracker.measure(f, now)
        self.m = m
        if m is None:
            return
        self.t_seen = now
        self.cmd = (self.tracker.yaw_rate(m), self.tracker.climb(m), self.forward(m, now))
        self.safe.rates(yaw_rate=self.cmd[0], climb=self.cmd[1], forward=self.cmd[2])
        det, r = m["det"], m["ray"]
        g = lambda k, n=2: "" if m.get(k) is None else f"{m[k]:.{n}f}"
        sticks = self.safe.command()
        self.rows.append([f"{f.t:.3f}", f"{f.t_arrival:.3f}", f.seq, f"{det.u:.1f}", f"{det.v:.1f}",
                          f"{(det.box[2] + det.box[3]) / 2:.1f}" if det.box else "", *(f"{x:.5f}" for x in r),
                          g("ang_h"), g("ang_v"), g("pred_h"), g("heading"), g("az"), g("az_f"), g("omega"),
                          g("range", 3), *(f"{x:.3f}" for x in self.cmd), *(f"{x:.3f}" for x in
                                                                            (sticks[0], sticks[1], sticks[3]))])

    def step(self, now):
        if self.waiting:
            self.safe.release()
            self.run.log_cmd(self.safe.command(), now)
            if now - self.t0 > self.WAIT_CLICK:
                self.run.event("no click: not started")
                return False
            return True
        if now - self.t_seen > self.LOST_AFTER:
            self.safe.release()             # target lost: hover
            self.tracker.reset()
            self.cmd = (0.0, 0.0, 0.0)
        else:
            self.safe.rates(yaw_rate=self.cmd[0], climb=self.cmd[1], forward=self.cmd[2])  # keep it alive
        self.tracker.note_command(now)
        self.run.log_cmd(self.safe.command(), now)
        return now - self.t0 < self.MAX_TIME

    def marker(self):
        return self.m["det"] if self.m else None

    def overlay(self):
        if self.waiting:
            return [f"CLICK the target to start (hovering, {self.WAIT_CLICK - (time.time() - self.t0):.0f} s left)"]
        if not self.m:
            return ["target: not seen"]
        m = self.m
        s = f"off-centre h {m['ang_h']:+.1f} (pred {m['pred_h']:+.1f}) v {m['ang_v']:+.1f} deg  target turns " \
            f"{m['omega']:+.1f} deg/s"
        return [s] + ([f"range {m['range']:.2f} m"] if m.get("range") else [])

    def report(self, states):
        rows = self.rows
        if len(rows) < 10:
            return ["follow: the target was hardly seen"]
        t = np.array([float(r[0]) for r in rows])
        ah = np.array([float(r[9]) for r in rows])
        av = np.array([float(r[10]) for r in rows])
        lines = [f"follow: target seen in {len(rows)} frames over {t[-1] - t[0]:.0f} s; off-centre |h| "
                 f"{np.mean(np.abs(ah)):.1f} deg (max {np.max(np.abs(ah)):.0f}), |v| {np.mean(np.abs(av)):.1f} deg"]
        w = window(states, t[0], t[-1])
        if len(w) > 20:
            ts = np.array([s.t for s in w])
            yaw = unwrap_deg([s.yaw for s in w])
            dy = np.interp(t, ts, yaw)
            ball = dy + ah
            grid = np.arange(t[0], t[-1], 0.05)
            b = np.interp(grid, t, ball) - ball.mean()
            y = np.interp(grid, t, dy) - dy.mean()
            if np.std(b) > 2:
                lag = max(range(0, 60), key=lambda L: float(np.dot(b[:len(b) - L], y[L:]))) * 0.05
                lines.append(f"  target moved (spread {np.std(ball):.1f} deg); drone heading spread {np.std(dy):.1f} "
                             f"deg, {lag:.2f} s behind the target")
            else:
                lines.append("  target stayed still")
        return lines


class Approach(Follow):
    key, name = "a", "approach"
    title = "APPROACH the ball: centre it, fly to it, stop at 0.5 m (range from its size, --ball-size)"
    STOP_AT = 0.5
    V_MIN = 0.2         # m/s, the slowest speed the sticks give (dead-zone edge)
    K_R = 0.5           # m/s per metre still to go
    BRAKE = 0.7         # s; braking ~ speed x this (sim: ~0.7 s; check with the outdoor speed table)
    TAU = 0.7           # s; the drone reaches a new speed command over about this long
    RESTART = 0.15      # m; once arrived, moves again only if this much farther than the stop distance
    AIM = 8.0           # deg; moves only while pointing at the target this well
    MAX_TIME = 45.0
    HOLD = 3.0          # s at the stop distance before the test ends

    def start(self, run):
        super().start(run)
        self.ranges = []
        self.t_arrived = None
        self.r_start = None
        self.state = "aiming"
        self.v_model, self.t_model = 0.0, None     # the drone's forward speed as the command model expects it

    def forward(self, m, now):
        if self.t_model is not None:            # speed model: follows the forward command with lag TAU
            self.v_model += (self.cmd[2] - self.v_model) * min(1.0, (now - self.t_model) / self.TAU)
        self.t_model = now
        if m.get("range") is None:
            return 0.0
        self.ranges = (self.ranges + [m["range"]])[-5:]
        r = float(np.median(self.ranges))
        if self.r_start is None and len(self.ranges) == 5:
            self.r_start = r
        # the range is one video delay old; the drone keeps going while the stick acts and while it brakes
        lag = self.ctx.drone.video_delay + ACT_DELAY + self.BRAKE
        to_go = r - self.v_model * lag - self.STOP_AT
        m["to_go"] = to_go
        if to_go <= 0.03 or (self.state == "arrived" and to_go < self.RESTART):
            self.state = "arrived"
            if self.t_arrived is None:
                self.t_arrived = now
            return 0.0
        self.t_arrived = None
        if abs(m["pred_h"]) > self.AIM:
            self.state = "aiming"
            return 0.0
        self.state = "approaching"
        return float(np.clip(max(self.V_MIN, self.K_R * to_go), 0.0, self.ctx.profile.approach_speed))

    def step(self, now):
        if now - self.t_seen > 0.2 and self.cmd[2] > 0:       # lost while moving: stop moving at once
            self.cmd = (self.cmd[0], self.cmd[1], 0.0)
        running = super().step(now)
        return running and not (self.t_arrived is not None and now - self.t_arrived > self.HOLD)

    def overlay(self):
        lines = super().overlay()
        if self.m and self.m.get("to_go") is not None:
            lines.append(f"{self.state}: {self.m['to_go']:+.2f} m to go")
        return lines

    def report(self, states):
        lines = super().report(states)
        rng = [(float(r[0]), float(r[16])) for r in self.rows if r[16]]
        if not rng:
            return lines + ["approach: no range (the target has no size in the image)"]
        t, r = np.array(rng).T
        final = r[t > t[-1] - 1.0]
        lines.append(f"approach: from {self.r_start or r[0]:.2f} m; final {np.median(final):.2f} m (target "
                     f"{self.STOP_AT} m), closest {r.min():.2f} m, {'arrived' if self.state == 'arrived' else 'NOT arrived'}"
                     f" after {t[-1] - t[0]:.0f} s")
        return lines


# ---------------- route: waypoints from a CSV, flown on GPS ----------------
ROUTE_COLUMNS = ["t", "action", "seg", "wp", "tgt_e", "tgt_n", "tgt_u", "est_e", "est_n", "est_u", "est_yaw",
                 "yaw", "up", "right", "forward"]


def load_route(path):
    """Waypoints of a route CSV (x north, y east, alt above takeoff, hold s), positions turned into ENU."""
    with open(path, newline="") as f:
        return [dict(wp=int(r["wp"]), seg=r["seg"], p=np.array([float(r["y"]), float(r["x"]), float(r["alt"])]),
                     hold=float(r["hold"])) for r in csv.DictReader(f)]


def compass(d):
    """Compass bearing (deg) of the ENU direction d."""
    return math.degrees(math.atan2(d[0], d[1]))


class Route(Test):
    """
    Fly part of the route CSV from where the test starts (GPS; alt = height above takeoff).
    CSV columns: wp (number), seg (<part>_<kind>, e.g. 2_step_full), x (m north of the start), y (m east),
    alt (m above takeoff), hold (s hovering at the point). Kinds:
      hover                       sticks centred for the hold
      step_full, step_half        fixed forward stick 1.0 / 0.5 (open loop: the step response), let go early
                                  enough to stop at the waypoint (the drone brakes itself)
      reversal                    full stick, reversed at the waypoint (backward stick: no turn)
      climb, descent              fixed full up / down stick
      yaw                         full yaw stick: 360 deg right, hold, 360 deg left, hold
      turn_*, free                follower: nose towards a point LOOK m ahead on the line, PATH_SPEED
    After a hold, a step first turns to face its waypoint (it is then flown nose first). Every part starts by
    flying to its first point and hovering there.
    """
    envs = ("route",)
    segments = ()               # segment-name prefixes of this part
    LOOK = 4.0                  # m
    PATH_SPEED = 1.3            # m/s (full stick is ~1.45 in the mode we measured)
    BRAKE = 0.6                 # s; braking distance ~ speed x this (measured 0.8 m from 1.45 m/s)
    BRAKE_UP = 0.3              # s
    BRAKE_YAW = 0.2             # s
    K_YAW = 2.0                 # deg/s per deg
    K_ALT = 0.8                 # 1/s
    MIN_YAW = 15.0              # deg/s; smaller turns are lost in the stick's dead zone (stick 0.2: ~3 deg/s outdoors)
    FACE_SETTLE = 1.5           # s; telemetry lags ~1 s
    STEP_ALIGN = 30.0           # deg; a step turns first only if the nose is further off its leg
    NEAR = 2.0                  # m; closer to a point the follower slides there without turning
    START_HOLD = 10.0           # s

    def plan(self):
        route = load_route(self.ctx.args.route)
        mine = [i for i, w in enumerate(route) if w["seg"].split("_", 1)[0] in self.segments]
        first = mine[0] if route[mine[0]]["seg"].endswith("hover") else mine[0] - 1
        start = route[first]
        acts = [dict(kind="path", a=None, b=start["p"], stop=True, w=start)]
        if start not in [route[i] for i in mine]:
            acts.append(dict(kind="hold", s=self.START_HOLD, w=start))
        prev, held = start["p"], True
        for i in mine:
            w = route[i]
            kind = w["seg"].split("_", 1)[1]
            if kind == "hover":
                pass
            elif kind in ("step_full", "step_half", "reversal"):
                if held:
                    acts.append(dict(kind="face", b=w["p"], w=w))
                acts.append(dict(kind="step", a=prev, b=w["p"], value=0.5 if kind == "step_half" else 1.0, w=w))
            elif kind in ("climb", "descent"):
                acts.append(dict(kind="climb", a=prev, b=w["p"], w=w))
            elif kind == "yaw":
                acts += [dict(kind="spin", turn=360.0, w=w), dict(kind="hold", s=w["hold"], w=w),
                         dict(kind="spin", turn=-360.0, w=w)]
            else:
                nxt = route[i + 1] if i + 1 < len(route) else None
                stop = w["hold"] > 0 or nxt is None or not nxt["seg"].split("_", 1)[1].startswith(("turn", "free"))
                acts.append(dict(kind="path", a=prev, b=w["p"], stop=stop, w=w))
            if w["hold"] > 0:
                acts.append(dict(kind="hold", s=w["hold"], w=w))
            held, prev = w["hold"] > 0, w["p"]
        return acts

    def can_start(self):
        why = super().can_start()
        if why:
            return why
        s = self.ctx.drone.latest_state()
        if s is None or s.pos_enu is None:
            return "no GPS position"
        if not os.path.exists(self.ctx.args.route):
            return f"no route file {self.ctx.args.route}"
        return None

    def start(self, run):
        super().start(run)
        self.acts = self.plan()
        self.origin = self.ctx.drone.latest_state().pos_enu[:2].copy()
        self.done = []              # (action, t_start, t_end, why)
        self.i = -1
        self.rows = run.table("route", ROUTE_COLUMNS)
        self.t_row = 0.0
        self.next_action(self.t0)

    def next_action(self, now):
        self.i += 1
        if self.i >= len(self.acts):
            return False
        act = self.acts[self.i]
        act["t0"] = now
        est, yaw, _ = self.estimate(now)
        if act["kind"] == "path" and act["a"] is None:
            act["a"] = est
        if act["kind"] == "step":
            # the step starts where the drone is (wind pushes it off the route's line): the turn before it aims from
            # here too. Measuring the leg from the route's point instead looped for ever on 3 Nov: the turn ended
            # aimed at the waypoint from here, the step found the nose 16-25 deg off the route's leg, turned again...
            act["a"] = np.r_[est[:2], act["a"][2]]
            # forward if the nose points along the leg, backward if against it (a reversal); turn first (once) if
            # neither. Loose (STEP_ALIGN): on 4 Nov the wind had put the drone 4 m off the line, the reversal's
            # leg was 14 deg off the nose, and the strict check turned it round instead of flying backward
            c = math.cos(math.radians(compass(act["b"] - act["a"]) - yaw))
            if abs(c) < math.cos(math.radians(self.STEP_ALIGN)) and not act.get("turned"):
                act["turned"] = True
                self.run.event(f"step to wp {act['w']['wp']}: nose {math.degrees(math.acos(abs(c))):.0f} deg off "
                               f"the leg, turning first")
                self.acts.insert(self.i, dict(kind="face", b=act["b"], w=act["w"]))
                self.i -= 1
                return self.next_action(now)
            act["sign"] = 1.0 if c > 0 else -1.0
        if act["kind"] == "spin":
            act["yaw0"] = yaw
        dist = float(np.linalg.norm(act["b"] - act["a"])) if "b" in act and act.get("a") is not None else 0.0
        act["timeout"] = act.get("s", 0.0) + 2.5 * dist / 0.4 + 20.0
        return True

    def estimate(self, now):
        """Position (ENU from the start, height) and velocity now (the late telemetry carried forward), and the
        predicted yaw."""
        s = self.ctx.drone.latest_state()
        p = np.r_[s.pos_enu[:2] - self.origin, s.height] if s.pos_enu is not None else np.zeros(3)
        return p + s.vel_enu * (now - s.t), self.predicted_yaw(s, now), s.vel_enu

    def predicted_yaw(self, s, now):
        """Yaw ACT_DELAY from now: the telemetry yaw plus the turn the sticks sent since will make (each answered
        ACT_DELAY later; as in Tracker). Steering on the late telemetry alone oscillates."""
        a = s.t - ACT_DELAY
        h = [x for x in self.safe.history if x[0] > a - 1.0]
        total = 0.0
        for (t0, cmd), (t1, _) in zip(h, h[1:] + [(now, None)]):
            lo, hi = max(t0, a), min(t1, now)
            if hi > lo:
                total += self.safe.expected_rates(cmd)[0] * (hi - lo)
        return s.yaw + total

    def yaw_rate(self):
        st = self.ctx.drone.states()[-4:]
        if len(st) < 2 or st[-1].t <= st[0].t:
            return 0.0
        return sum(wrap180(b.yaw - a.yaw) for a, b in zip(st, st[1:])) / (st[-1].t - st[0].t)

    def step(self, now):
        act = self.acts[self.i]
        est, yaw, vel = self.estimate(now)
        kind, why = act["kind"], None
        if kind == "hold":
            self.safe.release()
            why = "held" if now - act["t0"] >= act["s"] else None
        elif kind == "face":
            # done only after FACE_SETTLE s without a turn, when the predicted yaw is just the telemetry's: the
            # yaw map promises more than the drone gives outdoors (full stick 37 deg/s, not 47), so on 4 Nov the
            # turns ended 7-21 deg short. Still off then: turn again.
            err = wrap180(compass(act["b"] - est) - yaw)
            if abs(err) < 3.0:
                self.safe.release()
                act.setdefault("calm", now)
                why = "facing" if now - act["calm"] >= self.FACE_SETTLE else None
            else:
                act.pop("calm", None)
                self.safe.rates(yaw_rate=np.sign(err) * max(self.MIN_YAW, self.K_YAW * abs(err)))
        elif kind == "step":
            u = (act["b"] - act["a"])[:2]
            length = float(np.linalg.norm(u))
            u = u / length
            along = float((est[:2] - act["a"][:2]) @ u) + float(vel[:2] @ u) * (ACT_DELAY + self.BRAKE)
            if along >= length:
                self.safe.release()
                why = "reached"
            else:
                self.safe.sticks(forward=act["sign"] * act["value"])
        elif kind == "climb":
            dz = act["b"][2] - act["a"][2]
            done = (est[2] - act["a"][2]) * np.sign(dz) + abs(vel[2]) * (ACT_DELAY + self.BRAKE_UP)
            if done >= abs(dz):
                self.safe.release()
                why = "reached"
            else:
                self.safe.sticks(up=float(np.sign(dz)))
        elif kind == "spin":
            states = [s for s in self.ctx.drone.states() if s.t >= act["t0"]]
            turned = sum(wrap180(b.yaw - a.yaw) for a, b in zip(states, states[1:]))
            if states:
                turned += wrap180(states[0].yaw - act["yaw0"])
            ahead = self.yaw_rate() * ((now - states[-1].t if states else 0.0) + ACT_DELAY + self.BRAKE_YAW)
            if (turned + ahead) * np.sign(act["turn"]) >= abs(act["turn"]):
                self.safe.release()
                why = "turned"
            else:
                self.safe.sticks(yaw=float(np.sign(act["turn"])))
        else:
            why = self.follow(act, est, yaw)
        if why is None and now - act["t0"] > act["timeout"]:
            self.safe.release()
            why = "TIMEOUT"
        self.log(act, now, est, yaw)
        if why:
            self.done.append((act, act["t0"], now, why))
            if why == "TIMEOUT":
                self.run.event(f"{kind} to wp {act['w']['wp']} timed out")
            return self.next_action(now)
        return True

    def follow(self, act, est, yaw):
        a, b = act["a"], act["b"]
        u = (b - a)[:2]
        length = float(np.linalg.norm(u))
        u = u / length if length > 0.01 else np.zeros(2)
        along = float(np.clip((est[:2] - a[:2]) @ u, 0.0, length))
        carrot = a[:2] + u * min(length, along + self.LOOK)
        alt = a[2] + (b[2] - a[2]) * (along / length if length > 0.01 else 1.0)
        to_b = float(np.linalg.norm(b[:2] - est[:2]))
        if act["stop"] and to_b < 1.0 and abs(b[2] - est[2]) < 0.3:
            self.safe.release()
            return "reached"
        if not act["stop"] and along + self.LOOK >= length:
            return "passed"         # the next leg's carrot takes over without a stop
        climb = float(np.clip(self.K_ALT * (alt - est[2]), -0.5, 0.5))
        near = carrot - est[:2]
        if np.linalg.norm(near) < self.NEAR:
            # (nearly) straight up or down: slide to the point without turning. Aiming the nose at a point under
            # the drone spun it round and round at the start of the 4 Nov flights
            y = math.radians(yaw)
            right, forward = near[0] * math.cos(y) - near[1] * math.sin(y), near[0] * math.sin(y) + near[1] * math.cos(y)
            k = min(0.5, 0.5 / max(float(np.linalg.norm(near)), 1e-6))     # 0.5 m/s per m, 0.5 m/s at most
            self.safe.rates(climb=climb, right=k * right, forward=k * forward)
            return None
        err = wrap180(compass(carrot - est[:2]) - yaw)
        speed = self.PATH_SPEED * max(0.0, math.cos(math.radians(err)))
        if act["stop"]:
            speed = min(speed, 0.5 * to_b)
        self.safe.rates(yaw_rate=self.K_YAW * err, climb=climb, forward=speed)
        return None

    def log(self, act, now, est, yaw):
        self.run.log_cmd(self.safe.command(), now)
        if now - self.t_row < 0.1:
            return
        self.t_row = now
        tgt = act.get("b", est)
        self.rows.append([f"{now:.3f}", act["kind"], act["w"]["seg"], act["w"]["wp"],
                          *(f"{x:.2f}" for x in tgt), *(f"{x:.2f}" for x in est), f"{yaw:.1f}",
                          *(f"{x:.2f}" for x in self.safe.command())])

    def overlay(self):
        act = self.acts[min(self.i, len(self.acts) - 1)]
        return [f"action {self.i + 1}/{len(self.acts)}: {act['kind']} {act['w']['seg']} wp {act['w']['wp']}"]

    def report(self, states):
        st = [s for s in states if s.pos_enu is not None]
        if not st:
            return ["no GPS states"]
        t = np.array([s.t for s in st])
        p = np.array([np.r_[s.pos_enu[:2] - self.origin, s.height] for s in st])
        v = np.array([s.vel_enu for s in st])
        lines = [f"route part {self.name}: {len(self.done)}/{len(self.acts)} actions, "
                 f"{(self.done[-1][2] - self.t0) / 60 if self.done else 0:.1f} min"]
        for act, t0, t1, why in self.done:
            w = act["w"]
            m = (t >= t0) & (t <= t1 + 3.0)
            head = f"  {act['kind']:5s} {w['seg']:13s} wp {w['wp']:2d} {t1 - t0:5.1f} s {why:8s}"
            if act["kind"] == "step" and m.any():
                u = (act["b"] - act["a"])[:2]
                length = float(np.linalg.norm(u))
                u /= length
                along = (p[m, :2] - act["a"][:2]) @ u
                lines.append(head + f" peak {np.abs(v[m, :2] @ u).max():.2f} m/s, stopped at {along.max():.1f} "
                             f"of {length:.1f} m")
            elif act["kind"] == "climb" and m.any():
                lines.append(head + f" peak {np.abs(v[m, 2]).max():.2f} m/s, height {p[m, 2].min():.1f}-"
                             f"{p[m, 2].max():.1f} m (wanted {act['b'][2]:.1f})")
            elif act["kind"] == "path" and m.any() and np.linalg.norm((act["b"] - act["a"])[:2]) > 0.5:
                u = (act["b"] - act["a"])[:2]
                u /= np.linalg.norm(u)
                off = np.abs((p[m, :2] - act["a"][:2]) @ np.array([-u[1], u[0]]))
                lines.append(head + f" off the line {off.mean():.1f} m mean, {off.max():.1f} max; "
                             f"speed {np.linalg.norm(v[m, :2], axis=1).mean():.2f} m/s")
            elif act["kind"] == "hold" and m.any():
                lines.append(head + f" drift {np.ptp(p[m, 0]):.1f} x {np.ptp(p[m, 1]):.1f} m")
            else:
                lines.append(head)
        return lines


class RouteA(Route):
    key, name = "r", "routeA"
    title = "ROUTE part A (segments 1-6: hover, steps, reversals, climb, yaw; ~7 min)"
    segments = ("1", "2", "3", "4", "5", "6")


class RouteB(Route):
    key, name = "t", "routeB"
    title = "ROUTE part B (segments 7-9: wide circle, tight circle, free path; ~7 min)"
    segments = ("7", "8", "9")


TESTS = [Kill, Hang, Square, Follow, Approach, SlowForward, Combined, SpeedForward, SpeedRight, YawUp, Turns, Landmark,
         RouteA, RouteB]

PLAN = {
    "indoor": ["First: h hang (the watchdog must stop the turn), then 2 square, 3 follow, a approach, w slow table",
               "1 kill ends the program: fly it last"],
    "outdoor": ["Open space, GPS: 2 square, c two fingers, v max yaw + climb, o turns, 3 follow, l landmark",
                "s / d full speed tables need ~40 m clear"],
    "route": ["Open field, clear around the whole route, little wind. Take off, climb, wait for GPS.",
              "Start above the takeoff point (DJI stops the Neo ~50 m from it). r part A, t part B (new battery)"],
}
