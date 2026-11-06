"""
One test run: its files and the report lines every run gets.

Files, recordings/flight_<date>_<time>_<env>_<test>_*.csv:
  _cmd.csv       sticks applied, ~10 Hz
  _states.csv    telemetry from 2 s before to ~3 s after the run
  _<table>.csv   any extra table the test keeps (e.g. follow: one row per frame)
  _report.txt    the report printed after the run
"""
import csv
import math
import os
import time
from datetime import datetime

import numpy as np

AFTER = 3.0         # s of telemetry kept after a run (it arrives ~0.8 s late, in batches)
STATE_COLUMNS = ["t", "t_arrival", "yaw", "pitch", "roll", "gimbal_yaw", "gimbal_pitch", "vE", "vN", "vU", "height",
                 "ground", "pE", "pN", "gps_ok", "gps_num", "battery", "mode"]


def fmt(x, n=3):
    return "" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{n}f}"


class Run:
    def __init__(self, out_dir, env, name):
        self.name = name
        os.makedirs(out_dir, exist_ok=True)
        self.base = os.path.join(out_dir, f"flight_{datetime.now():%Y%m%d_%H%M%S}_{env}_{name}")
        self.t_start = time.time()
        self.t_end = None
        self.sent = []
        self.tables = {}            # name -> (columns, rows)
        self.events = []            # (time, text): aborts, warnings

    def log_cmd(self, cmd, now):
        if not self.sent or cmd != self.sent[-1][1:] or now - self.sent[-1][0] > 0.1:
            self.sent.append((now, *cmd))

    def table(self, name, columns):
        """Rows list of an extra table (created on first use)."""
        if name not in self.tables:
            self.tables[name] = (columns, [])
        return self.tables[name][1]

    def event(self, text):
        self.events.append((time.time(), text))
        print(f"  [{self.name}] {text}")

    def states_of(self, states):
        t_end = self.t_end or time.time()
        return [s for s in states if self.t_start - 2 <= s.t <= t_end + AFTER]

    def save(self, states, report=None):
        files = [self.base + "_cmd.csv", self.base + "_states.csv"]
        if report:
            files.append(self.base + "_report.txt")
            with open(files[-1], "w", encoding="utf-8") as f:
                f.writelines(line + "\n" for line in report)
        with open(files[0], "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "yaw", "up", "right", "forward"])
            w.writerows([f"{r[0]:.3f}", *(f"{x:.3f}" for x in r[1:])] for r in self.sent)
        with open(files[1], "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(STATE_COLUMNS)
            for s in self.states_of(states):
                p = s.pos_enu if s.pos_enu is not None else (None, None)
                w.writerow([f"{s.t:.3f}", f"{s.t_arrival:.3f}", s.yaw, s.pitch, s.roll, s.gimbal_yaw, s.gimbal_pitch,
                            *(f"{v:.3f}" for v in s.vel_enu), s.height, fmt(s.ground, 2), fmt(p[0], 2), fmt(p[1], 2),
                            int(s.gps_ok), s.raw.get("gpsNum", ""), s.battery, s.mode])
        for name, (columns, rows) in self.tables.items():
            files.append(f"{self.base}_{name}.csv")
            with open(files[-1], "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(columns)
                w.writerows(rows)
        if self.events:
            files.append(self.base + "_events.txt")
            with open(files[-1], "w") as f:
                f.writelines(f"{t - self.t_start:7.2f} s  {text}\n" for t, text in self.events)
        print("  " + "\n  ".join(files))


def common_report(run, states):
    """Lines every run reports: wind (hover tilt before the start), heights, battery, GPS."""
    lines = []
    before = [s for s in states if run.t_start - 2.0 <= s.t <= run.t_start]
    if before:
        # the drone leans into the wind to hold its place: lean = nose down (-pitch) forward, roll right
        f = -np.mean([s.pitch for s in before])
        r = np.mean([s.roll for s in before])
        y = math.radians(np.mean([s.yaw for s in before]))
        east, north = f * math.sin(y) + r * math.cos(y), f * math.cos(y) - r * math.sin(y)
        tilt = math.hypot(east, north)
        frm = math.degrees(math.atan2(east, north)) % 360
        lines.append(f"wind (hover tilt before start): {tilt:.1f} deg, from {frm:.0f} deg compass"
                     + ("  (calm)" if tilt < 1.5 else "  (noticeable)" if tilt < 5 else "  (STRONG)"))
    mine = run.states_of(states)
    if mine:
        h = [s.height for s in mine]
        g = [s.ground for s in mine if s.ground is not None]
        lines.append(f"height above takeoff {min(h):.1f}-{max(h):.1f} m; ground sensor "
                     + (f"{min(g):.1f}-{max(g):.1f} m" if g else "no reading"))
        sats = [s.raw.get("gpsNum") for s in mine if s.raw.get("gpsNum") is not None]
        lines.append(f"battery {mine[0].battery:.0f} -> {mine[-1].battery:.0f} %; GPS "
                     + (f"{min(sats)}-{max(sats)} sats, used {np.mean([s.gps_ok for s in mine]) * 100:.0f}% of the time"
                        if sats else "no data"))
    for t, text in run.events:
        lines.append(f"event at {t - run.t_start:.1f} s: {text}")
    return lines
