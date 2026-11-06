"""
DJI Neo adapter: phone (DJI Fly) -> PC.

  video      scrcpy copy of the phone screen (neo_pilot/phone.py),
             cropped to the camera area. Overlays of DJI Fly are still on it.
             Landscape camera view: ~1916x1080. Portrait Manual Control screen
             (the one with the joysticks): ~1080x608, so fx scales by 1080/1920.
  telemetry  DJI Fly's flight log, read live off the phone and decrypted
             (telemetry_js/live_log.mjs via neo_pilot/telemetry.py), 10 Hz.

Timing:
  frame.t    arrival time - VIDEO_DELAY (neo_maps.py). Measured with tools/video_latency.py:
             288-311 ms on the ground, 416 ms median while hovering (323-588 ms); turns
             in flight gave 0.566 s against the telemetry, which is used.
  state.t    the log writes one row every 0.1 s, so the row of slot k (its flyTime / 0.1; the phone sometimes
             skips rows) was made at t0 + 0.1 k. Every row gives an upper bound on t0: the time the
             phone stamped it (CUSTOM.dateTime, moved to the PC clock) or, failing
             that, when the PC got it, minus 0.1 k. t0 = the lowest bound so far.
             A stamp more than MAX_STAMP_ERROR from its arrival is ignored (the first
             row is sometimes 1970 or 2017: on 2026-11-03 a 2017 one put every
             state 9.5 years in the past).
             It settles within the first seconds; earlier states are shifted when
             it improves. Small radio delay drone -> phone is not removed.

    from neo_pilot.drone.neo import NeoDrone
    with NeoDrone() as d:
        d.wait_ready()
        f, s = d.latest_frame(), d.latest_state()
"""
import math
import random
import threading
import time
from datetime import datetime

import numpy as np

from ..phone import connect, video_alive
from ..telemetry import LiveTelemetry
from .base import Drone, Frame, State
from .neo_maps import SPEED_MAP, UP_MAP, VIDEO_DELAY, YAW_MAP
from .touch_guard import TouchGuard

CAMERA_X = (218 / 2456, 2134 / 2456)    # camera area in the landscape phone screen, as fractions of the width,
                                        # with the phone's navigation bar on the right (mirrored when on the left)
CAMERA_Y = (259 / 2414, 867 / 2414)     # camera area in the portrait "Manual Control" screen, fractions of the
                                        # height (from the DJI-Neo-to-PC example phone; not checked)
ROW_PERIOD = 0.1                # s between flight-log rows
MAX_STAMP_ERROR = 60.0          # s; a row's phone stamp further than this from its PC arrival is bogus

# Virtual joysticks in the landscape camera view (shown only while flying). Measured on cap_20261030_134931:
# stick centres as fractions of the camera area, ring radius in screen pixels. The joystick starts where the
# finger lands, so the centres only need to be inside the circles.
STICK_X = (316 / 1916, 1600 / 1916)     # left, right
STICK_Y = 784 / 1080
STICK_RADIUS = 108                      # full deflection, screen px. Measured 2026-10-31 from the flight log's RC
                                        # values: 215 px (the guess before) gave DJI twice the stick sent
MAX_STICK = 0.5                         # default cap on any stick (a flight profile can change max_stick)
CMD_TIMEOUT = 0.3                       # s; a command not renewed within this is dropped (sticks released)
SEND_HZ = 20
EARTH_R = 6378137.0


class NeoDrone(Drone):
    video_delay = VIDEO_DELAY
    max_stick = MAX_STICK
    YAW_MAP, UP_MAP, SPEED_MAP = YAW_MAP, UP_MAP, SPEED_MAP

    def __init__(self, existing_log=True, on_event=print):
        """
        existing_log: True = follow the newest flight log on the phone (the drone
        is already connected). False = wait for DJI Fly to start a new one.
        """
        super().__init__()
        self._states = []
        self._lock = threading.Lock()
        self._t0 = math.inf             # PC time of log row 0
        self._slot_shift = 0            # added to flyTime slots after a flyTime restart
        self._phone_minus_pc = 0.0
        self._home = None               # (lat, lon) of the first GPS fix
        self._seq = 0
        self._nav_left = False          # phone navigation bar on the left screen edge (phone turned the other way)
        self._camera_box = None         # (x0, x1, height) of the camera area in the landscape screen
        self._screen = None             # (width, height) of the phone screen
        self._stop = False

        # sticks: newest command, when it was given, which virtual sticks are pressed
        self._cmd = (0.0, 0.0, 0.0, 0.0)
        self._cmd_time = 0.0
        self._down = {"L": None, "R": None}     # touch point (x, y) while pressed
        self._pointer = {"L": random.getrandbits(62), "R": random.getrandbits(62)}
        self._touch_lock = threading.Lock()
        self._frozen_until = 0.0        # hang test: the sender pauses (no sticks, no heartbeat) until then

        self._tel = LiveTelemetry(existing=existing_log, on_event=self._event(on_event))
        self._device = connect()
        self._guard = TouchGuard()      # releases held sticks if this program dies
        self._device.registerVideoListener(self._on_screen)
        self._thread = threading.Thread(target=self._poll_telemetry, daemon=True)
        self._thread.start()
        self._sender = threading.Thread(target=self._send_sticks, daemon=True)
        self._sender.start()

    # ---- control: DJI Fly's virtual joysticks ----
    def sticks(self, yaw=0.0, up=0.0, right=0.0, forward=0.0):
        """
        Set the virtual sticks, each -1..1 (clipped to +-max_stick). Like a remote
        controller in DJI Fly's normal mode: left stick = yaw (right +) and up/down,
        right stick = right/left and forward/back, in the drone's own frame.

        The command holds for CMD_TIMEOUT only: call again (e.g. every 0.1 s) to
        keep it. If the calls stop, the sticks are released and the drone brakes
        and hovers. Only works while flying, in the landscape camera view.
        """
        c = lambda x: float(np.clip(x, -self.max_stick, self.max_stick))
        self._cmd = (c(yaw), c(up), c(right), c(forward))
        self._cmd_time = time.time()

    def release(self):
        """Let go of both sticks now."""
        self._cmd = (0.0, 0.0, 0.0, 0.0)
        self._cmd_time = 0.0
        with self._touch_lock:
            self._apply((0.0, 0.0, 0.0, 0.0))

    def command(self):
        """The stick values being applied now (zeros after the timeout)."""
        return self._cmd if time.time() - self._cmd_time <= CMD_TIMEOUT else (0.0, 0.0, 0.0, 0.0)

    def freeze_for_test(self, seconds):
        """Hang test: the whole program stops for `seconds` (this call blocks, and the stick sender
        pauses without releasing or sending its heartbeat), as if the PC froze with a stick held."""
        self._frozen_until = time.time() + seconds
        time.sleep(seconds)

    def _send_sticks(self):
        last = time.time()
        while not self._stop:
            if time.time() < self._frozen_until:
                time.sleep(0.02)
                last = time.time()
                continue
            if time.time() - last > 0.5:
                print(f"stick sender stalled {time.time() - last:.1f} s")
            last = time.time()
            self._guard.heartbeat()
            with self._touch_lock:
                try:
                    self._apply(self.command())
                except Exception as e:      # socket gone etc.: nothing more to send
                    print(f"stick sender stopped: {e}")
                    return
            time.sleep(1 / SEND_HZ)

    def _apply(self, cmd):
        yaw, up, right, forward = cmd
        box = self._camera_box
        for side, (dx, dy) in (("L", (yaw, -up)), ("R", (right, -forward))):
            if dx == 0 and dy == 0 or box is None:
                if self._down[side] is not None:
                    self._device.touchUp(self._pointer[side], *self._down[side])
                    self._guard.released(side)
                    self._down[side] = None
                continue
            x0, x1, h = box
            fx = STICK_X[0] if side == "L" else STICK_X[1]
            cx, cy = int(x0 + fx * (x1 - x0)), int(STICK_Y * h)
            if self._down[side] is None:        # the joystick starts where the finger lands
                self._device.touchDown(self._pointer[side], cx, cy)
                self._guard.pressed(side, self._pointer[side], cx, cy, *self._screen)
                self._down[side] = (cx, cy)
            cx, cy = self._down[side]
            self._device.touchMove(self._pointer[side], int(cx + dx * STICK_RADIUS), int(cy + dy * STICK_RADIUS))

    # ---- video ----
    def _on_screen(self, screen):
        t = time.time()
        h, w = screen.shape[:2]
        self._screen = (w, h)
        if w > h:       # landscape: full-screen camera view
            # The phone's black navigation bar is on the right or the left, depending on which way the phone
            # is turned; the camera area is mirrored with it. The bar's side = the darker screen edge.
            left, right = screen[::8, :16].mean(), screen[::8, -16:].mean()
            if abs(left - right) > 10:
                self._nav_left = left < right
            x0, x1 = int(CAMERA_X[0] * w), int(CAMERA_X[1] * w)
            if self._nav_left:
                x0, x1 = w - x1, w - x0
            image = screen[:, x0:x1]
            self._camera_box = (x0, x1, h)
        else:           # portrait: Manual Control screen (joysticks), small camera strip on top
            image = screen[int(CAMERA_Y[0] * h):int(CAMERA_Y[1] * h)]
            self._camera_box = None
        self._new_frame(Frame(image=image, t=t - self.video_delay, t_arrival=t, seq=self._seq))
        self._seq += 1

    # ---- telemetry ----
    def _event(self, user_cb):
        def cb(ev):
            if ev["event"] == "clock":
                self._phone_minus_pc = ev["phone_minus_pc"]
            elif ev["event"] == "new_log":     # new log = row count restarts
                with self._lock:
                    self._states = []
                    self._t0 = math.inf
                    self._slot_shift = 0
                    self._home = None
            if user_cb:
                user_cb(ev)
        return cb

    def _poll_telemetry(self):
        while not self._stop:
            with self._lock:
                n = len(self._states)
            new = self._tel.rows_since(n)
            if not new:
                time.sleep(0.05)
                continue
            with self._lock:
                if len(self._states) != n:      # a new log started meanwhile
                    continue
                for k, row in enumerate(new, start=n):
                    self._add_row(k, row)

    def _add_row(self, k, row):
        bound = row["t_pc"]
        stamped = datetime.fromisoformat(row["dateTime"].replace("Z", "+00:00")).timestamp() - self._phone_minus_pc
        if abs(stamped - row["t_pc"]) < MAX_STAMP_ERROR:     # first rows can be 1970 or 2017 (2026-11-03): skip
            bound = min(bound, stamped)
        # the row's slot from the drone's flyTime, not its count: the phone sometimes skips rows (2026-11-04: 11.5 s
        # of rows lost in a 213 s flight), and counting them put every later state that much too early
        fly = row.get("flyTime")
        if fly is not None:
            last = self._states[-1].seq if self._states else -1
            k = round(fly / ROW_PERIOD) + self._slot_shift
            if k <= last:                   # flyTime restarted (a second takeoff in the same log): carry on
                self._slot_shift += last + 1 - k
                k = last + 1
        t0 = bound - ROW_PERIOD * k
        if t0 < self._t0:
            if self._states:        # move the earlier states to the better estimate
                shift = t0 - self._t0
                for s in self._states:
                    s.t += shift
            self._t0 = t0

        gps_ok = bool(row["isGPSUsed"]) and row["lat"] != 0
        pos = None
        if gps_ok:
            if self._home is None:
                self._home = (row["lat"], row["lon"])
            lat0, lon0 = self._home
            east = math.radians(row["lon"] - lon0) * EARTH_R * math.cos(math.radians(lat0))
            north = math.radians(row["lat"] - lat0) * EARTH_R
            pos = np.array([east, north, row["height"]])
        self._states.append(State(
            t=self._t0 + ROW_PERIOD * k, t_arrival=row["t_pc"], seq=k,
            pos_enu=pos, vel_enu=np.array([row["vE"], row["vN"], -row["vD"]]),
            height=row["height"], roll=row["roll"], pitch=row["pitch"], yaw=row["yaw"],
            gimbal_pitch=row["gimbalPitch"], gimbal_yaw=row["gimbalYaw"],
            lat=row["lat"] if gps_ok else None, lon=row["lon"] if gps_ok else None,
            gps_ok=gps_ok, motors_on=bool(row["isMotorOn"]), mode=row["flycState"],
            battery=row["battery"], raw=row,
            ground=row["vpsHeight"] if row.get("vpsHeight") else None))     # 0 = no reading (not checked yet)

    # ---- Drone interface ----
    def states(self):
        with self._lock:
            return list(self._states)

    def alive(self):
        return video_alive(self._device) and self._tel.alive()

    def close(self):
        try:
            self.release()
        except Exception:
            pass
        self._stop = True
        self._guard.close()
        self._device.removeVideoListener()
        self._device.destroy()
        self._tel.close()
