"""
Simulated drone with the Neo's measured behaviour, to rehearse flight programs on the PC.

What it copies from the real Neo (2026-10-30/31 measurements):
  video       frames reach the PC 0.42 s + extra_video_delay after capture (the extra is the
              unexplained frame/telemetry offset; the adapter believes 0.42 s, like NeoDrone)
  telemetry   10 Hz rows, arriving ~0.75 s late in batches every 0.7 s
  sticks      0.25 s until the drone answers, dead zones, the measured stick -> rate maps;
              (real stick units, DJI Fly's Cine mode: full stick ~1.5 m/s, 47 deg/s,
              0.5 m/s climb); commands expire after 0.3 s
  dynamics    yaw rate settles in ~0.15 s, horizontal speed in ~0.7 s, climb ~0.3 s
  heights     height above the takeoff point (indoors: a 0.7 m table), ground = to the floor
  wind        outdoors: the drone holds against it and leans into it
  watchdog    freeze_for_test(): the held stick is released 1 s into the freeze
  pop-up      popup(seconds): the app ignores the sticks (like DJI Fly's warnings)

The scene: a grey textured panorama around the drone and a tennis ball (still, sweeping
left/right, or walking along a line), drawn with a nominal lens (LENS_FX, no distortion).
"""
import math
import random
import threading
import time
from collections import deque

import cv2
import numpy as np

from .base import Drone, Frame, State, forward_map
from .neo_maps import SPEED_MAP, UP_MAP, VIDEO_DELAY, YAW_MAP

SIZE = (1916, 1080)
LENS_FX = 1600.0        # px for SIZE: the live view in DJI Fly (turns test o: 1594-1614 px; field of view ~62 deg)
BALL_COLOUR_HSV = (35, 171, 241)

# the drone's "truth" in real stick units, Cine mode as measured 2026-10-31 (full stick: ~47 deg/s, 0.5 m/s, ~1.5 m/s)
TRUE_YAW = [(0.20, 5.0), (0.30, 16.0), (0.40, 22.0), (0.60, 31.0), (0.80, 42.0), (1.00, 47.0)]
TRUE_UP = [(0.40, 0.10), (0.60, 0.18), (0.80, 0.26), (1.00, 0.48)]
TRUE_SPEED = [(0.30, 0.20), (0.40, 0.28), (0.60, 0.53), (0.80, 0.79), (1.00, 1.50)]

ACT = 0.25
CMD_TIMEOUT = 0.3
TEL_LAG, TEL_BATCH = 0.75, 0.7
DT = 0.02


def rotation_ned(yaw, pitch, roll):
    """Body FRD -> NED, angles in degrees (Z-Y-X order)."""
    y, p, r = np.radians([yaw, pitch, roll])
    cy, sy, cp, sp, cr, sr = np.cos(y), np.sin(y), np.cos(p), np.sin(p), np.cos(r), np.sin(r)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


class SimDrone(Drone):
    video_delay = VIDEO_DELAY
    max_stick = 0.5
    YAW_MAP, UP_MAP, SPEED_MAP = YAW_MAP, UP_MAP, SPEED_MAP

    def __init__(self, env="indoor", ball="still", wind=(0.0, 0.0), extra_video_delay=0.1, seed=1):
        super().__init__()
        rng = np.random.default_rng(seed)
        random.seed(seed)
        self.env = env
        self.outdoor = env != "indoor"
        self.wind = np.array(wind if self.outdoor else (0.0, 0.0), float)
        self.true_video_delay = VIDEO_DELAY + extra_video_delay
        self.K = np.array([[LENS_FX, 0, SIZE[0] / 2], [0, LENS_FX, SIZE[1] / 2], [0, 0, 1]])
        self.dist = np.zeros(5)
        K = self.K
        self.ppd = K[0, 0] * math.pi / 180          # panorama pixels per degree of yaw

        # panorama: smooth grey noise, low saturation (so only the ball is "coloured")
        P = int(360 * self.ppd)
        small = rng.uniform(40, 200, (SIZE[1] // 12, P // 12)).astype(np.float32)
        pano = cv2.resize(small, (P, SIZE[1]), interpolation=cv2.INTER_CUBIC)
        pano += rng.normal(0, 8, pano.shape).astype(np.float32)
        self.pano = cv2.cvtColor(np.clip(pano, 0, 255).astype(np.uint8), cv2.COLOR_GRAY2BGR)
        self.ball_bgr = tuple(int(x) for x in cv2.cvtColor(np.uint8([[BALL_COLOUR_HSV]]), cv2.COLOR_HSV2BGR)[0, 0])

        # true state
        self.takeoff_z = 0.0 if self.outdoor else 0.7
        self.pos = np.array([0.0, 0.0, 1.5 if self.outdoor else 1.2])      # ENU, z above the floor/ground
        self.vel = np.zeros(3)
        self.yaw, self.yaw_rate = 30.0, 0.0
        self.roll = self.pitch = 0.0
        self.ball_mode = ball
        y = math.radians(self.yaw + 10)
        dist = 8.0 if self.outdoor else 2.5
        self.ball0 = np.array([dist * math.sin(y), dist * math.cos(y), self.pos[2] - 0.2])
        self.t_start = time.time()

        self._cmd, self._cmd_time = (0.0, 0.0, 0.0, 0.0), 0.0
        self._applied = deque()           # (time, sticks as the drone saw them)
        self._freeze = None                # (start, end, held sticks)
        self._popup_until = 0.0
        self._past = deque(maxlen=400)     # (t, yaw, roll, pos) for rendering late frames
        self._rows = []
        self._lock = threading.Lock()
        self._stop = False
        self._seq = 0
        threading.Thread(target=self._physics, daemon=True).start()
        threading.Thread(target=self._video, daemon=True).start()

    # ---- control ----
    def sticks(self, yaw=0.0, up=0.0, right=0.0, forward=0.0):
        c = lambda x: float(np.clip(x, -self.max_stick, self.max_stick))
        self._cmd, self._cmd_time = (c(yaw), c(up), c(right), c(forward)), time.time()

    def release(self):
        self._cmd, self._cmd_time = (0.0, 0.0, 0.0, 0.0), 0.0

    def command(self):
        return self._cmd if time.time() - self._cmd_time <= CMD_TIMEOUT else (0.0, 0.0, 0.0, 0.0)

    def freeze_for_test(self, seconds):
        now = time.time()
        self._freeze = (now, now + seconds, self.command())
        time.sleep(seconds)

    def popup(self, seconds):
        self._popup_until = time.time() + seconds

    def _touching(self, now):
        """Sticks the phone is holding now (a frozen program's held stick until the watchdog lets go)."""
        if now < self._popup_until:
            return (0.0, 0.0, 0.0, 0.0)
        if self._freeze and self._freeze[0] <= now < self._freeze[1]:
            return self._freeze[2] if now < self._freeze[0] + 1.0 else (0.0, 0.0, 0.0, 0.0)
        return self.command()

    # ---- the world ----
    def ball(self, t):
        k = t - self.t_start
        if self.ball_mode == "sweep":       # left/right on an arc around the start, +-25 deg, 8 s period
            az = math.radians(self.yaw0_ball() + 25 * math.sin(2 * math.pi * k / 8))
            r = np.linalg.norm(self.ball0[:2])
            return np.array([r * math.sin(az), r * math.cos(az), self.ball0[2]])
        if self.ball_mode == "walk":        # a person walking 1 m/s across, turning every 10 s
            phase = (k % 20) / 10
            s = phase if phase < 1 else 2 - phase
            y = math.radians(self.yaw0_ball() + 90)
            return self.ball0 + (s - 0.5) * 10 * np.array([math.sin(y), math.cos(y), 0])
        return self.ball0

    def yaw0_ball(self):
        return math.degrees(math.atan2(self.ball0[0], self.ball0[1]))

    def _physics(self):
        t_row = time.time()
        k = 0
        while not self._stop:
            now = time.time()
            self._applied.append((now, self._touching(now)))
            while self._applied and self._applied[0][0] < now - ACT - 0.5:
                self._applied.popleft()
            past = [c for t, c in self._applied if t <= now - ACT]
            yaw_s, up_s, right_s, fwd_s = past[-1] if past else (0.0, 0.0, 0.0, 0.0)

            self.yaw_rate += (forward_map(TRUE_YAW, yaw_s) - self.yaw_rate) * DT / 0.15
            self.yaw = (self.yaw + self.yaw_rate * DT + 180) % 360 - 180
            y = math.radians(self.yaw)
            fwd, right = forward_map(TRUE_SPEED, fwd_s), forward_map(TRUE_SPEED, right_s)
            want = np.array([fwd * math.sin(y) + right * math.cos(y), fwd * math.cos(y) - right * math.sin(y)])
            moving = min(1.0, float(np.linalg.norm(want)))
            want += 0.1 * self.wind * moving                # holds its place in wind; drifts a bit while moving
            acc = (want - self.vel[:2]) / 0.7
            acc = acc * min(1.0, 5.0 / max(np.linalg.norm(acc), 1e-6))     # at most 5 m/s^2
            self.vel[:2] += acc * DT
            self.vel[2] += (forward_map(TRUE_UP, up_s) - self.vel[2]) * DT / 0.3
            self.pos += self.vel * DT
            if self.pos[2] < 0.05:
                self.pos[2], self.vel[2] = 0.05, 0.0
            lean = 4.0 * acc - 1.5 * self.wind              # deg; leans into the wind and the acceleration
            self.pitch = -float(lean @ [math.sin(y), math.cos(y)])
            self.roll = float(lean @ [math.cos(y), -math.sin(y)])
            self._past.append((now, self.yaw, self.roll, self.pos.copy()))

            if now >= t_row:
                arrival = (math.floor((now + TEL_LAG) / TEL_BATCH) + 1) * TEL_BATCH
                gps = self.pos + np.r_[np.random.normal(0, 0.3, 2), 0] if self.outdoor else None
                st = State(t=now, t_arrival=arrival, seq=k, pos_enu=gps, vel_enu=np.round(self.vel, 2),
                           height=round(float(self.pos[2] - self.takeoff_z), 1), roll=self.roll, pitch=self.pitch,
                           yaw=round(self.yaw, 1), gimbal_pitch=0.0, gimbal_yaw=round(self.yaw, 1),
                           lat=None, lon=None, gps_ok=self.outdoor, motors_on=True, mode="GPSAtti",
                           battery=max(0.0, 90 - (now - self.t_start) / 12), raw={"gpsNum": 14 if self.outdoor else 0},
                           ground=round(float(self.pos[2]), 1))
                with self._lock:
                    self._rows.append(st)
                k += 1
                t_row += 0.1
            time.sleep(DT)

    def _video(self):
        P = self.pano.shape[1]
        while not self._stop:
            now = time.time()
            tc = now - self.true_video_delay
            past = [p for p in self._past if p[0] <= tc]
            if not past:
                time.sleep(0.04)
                continue
            _, yaw, roll, pos = past[-1]
            x0 = int(((yaw - SIZE[0] / 2 / self.ppd) % 360) * self.ppd)
            cols = (np.arange(SIZE[0]) + x0) % P
            img = self.pano[:, cols].copy()
            rel = self.ball(tc) - pos
            ned = np.array([rel[1], rel[0], -rel[2]])
            frd = rotation_ned(yaw, 0.0, roll).T @ ned
            if frd[0] > 0.2:
                pt, _ = cv2.projectPoints(np.array([[frd[1], frd[2], frd[0]]]), np.zeros(3), np.zeros(3),
                                          self.K, self.dist)
                u, v = pt.ravel()
                radius = max(2, int(self.K[0, 0] * 0.0335 / frd[0]))
                if -radius < u < SIZE[0] + radius and -radius < v < SIZE[1] + radius:
                    cv2.circle(img, (int(u), int(v)), radius, self.ball_bgr, -1, cv2.LINE_AA)
            self._new_frame(Frame(image=img, t=now - self.video_delay, t_arrival=now, seq=self._seq))
            self._seq += 1
            time.sleep(max(0.0, 0.04 - (time.time() - now)))

    # ---- Drone interface ----
    def states(self):
        now = time.time()
        with self._lock:
            return [s for s in self._rows if s.t_arrival <= now]

    def alive(self):
        return not self._stop

    def close(self):
        self.release()
        self._stop = True
