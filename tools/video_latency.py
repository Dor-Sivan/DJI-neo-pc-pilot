"""
End-to-end video delay: drone camera -> phone (DJI Fly) -> PC.

Method (no OCR, one clock):
  - The PC shows a full-screen window that switches between black and white at
    random moments, and notes the time of each switch (PC clock).
  - The drone camera looks at that monitor. In the phone video coming back, the
    brightness of the image centre jumps when the switch arrives.
  - delay = arrival time of the first frame past the halfway brightness - switch time.

Both times come from time.perf_counter() on this PC, so no clock sync is needed.
The result includes the monitor's own display lag (a few ms to ~30 ms), so it is
slightly pessimistic.

Setup:
  - Drone on the table (motors off), camera 20-50 cm from the monitor, so the
    monitor fills the middle of the DJI Fly view. Watch the phone to aim it.
  - DJI Fly on the live camera view.

Run:   python tools/video_latency.py
       python tools/video_latency.py --telemetry   also run the live flight-log reader
                                                             (adb busy as in a real flight)
Stop:  it ends by itself after N_FLASHES; q or Esc in the flash window stops early.
Writes recordings/latency_<date>_<time>[_tel]_frames.csv and _flashes.csv.
"""
import contextlib
import csv
import os
import random
import sys
import time
from datetime import datetime

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from neo_pilot.paths import RECORDINGS as OUT_DIR       # noqa: E402
from neo_pilot.phone import connect                     # noqa: E402
from neo_pilot.telemetry import LiveTelemetry           # noqa: E402

WITH_TELEMETRY = "--telemetry" in sys.argv

N_FLASHES = 20
GAP = (1.5, 2.5)        # random seconds between switches (random, so nothing periodic can lock onto it)
ROI = 0.3               # centre fraction of the phone screen used for brightness
SETTLE = 0.3            # seconds of signal before a switch used as "steady level"
MIN_CONTRAST = 20.0     # brightness jump (0-255) needed to trust a measurement
WIN = "flash (q to stop)"

samples = []    # (arrival time, centre brightness), one per phone frame


def on_frame(frame):
    t = time.perf_counter()
    h, w = frame.shape[:2]
    y0, x0 = int(h * (1 - ROI) / 2), int(w * (1 - ROI) / 2)
    samples.append((t, float(frame[y0:h - y0:4, x0:w - x0:4].mean())))


def measure(t0, t_next, to_white):
    """
    Delay of the switch at t0, using the frames up to the next switch at t_next.
    Returns (delay_s or None, contrast).
    """
    s = np.array(list(samples))
    t, b = s[:, 0], s[:, 1]
    before = b[(t > t0 - SETTLE) & (t <= t0)]
    settled = b[(t > t_next - SETTLE) & (t <= t_next)]
    if len(before) < 3 or len(settled) < 3:
        return None, 0.0
    lo, hi = np.median(before), np.median(settled)
    contrast = hi - lo
    if (contrast if to_white else -contrast) < MIN_CONTRAST:
        return None, contrast
    mid = (lo + hi) / 2
    window = (t > t0) & (t <= t_next)
    crossed = window & ((b > mid) if to_white else (b < mid))
    idx = np.flatnonzero(crossed)
    if len(idx) == 0:
        return None, contrast
    return t[idx[0]] - t0, contrast


def hold(seconds):
    """Keep the window responsive for `seconds`. False if the user pressed q/Esc."""
    deadline = time.perf_counter() + seconds
    while time.perf_counter() < deadline:
        if cv2.waitKey(10) & 0xFF in (ord("q"), 27):
            return False
    return True


black = np.zeros((360, 640, 3), np.uint8)
white = np.full_like(black, 255)
flashes = []    # (switch time, to_white, delay or None, contrast)


def report(k, t_next):
    t0, to_white = flashes[k][:2]
    delay, contrast = measure(t0, t_next, to_white)
    flashes[k] = (t0, to_white, delay, contrast)
    what = "black->white" if to_white else "white->black"
    if delay is None:
        print(f"flash {k + 1:2d} {what}: not detected (contrast {contrast:+.0f}, need {MIN_CONTRAST:.0f})")
    else:
        print(f"flash {k + 1:2d} {what}: delay {delay * 1000:5.0f} ms  (contrast {contrast:+.0f})")


with (LiveTelemetry(existing=True) if WITH_TELEMETRY else contextlib.nullcontext()) as tel, connect() as device:
    if WITH_TELEMETRY:
        print("live flight-log reader running in the background")
    device.registerVideoListener(on_frame)

    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(WIN, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    cv2.imshow(WIN, black)
    print("aim the drone camera at the monitor; starting in 3 s")
    running = hold(3.0)

    to_white = False
    for i in range(N_FLASHES):
        if not running:
            break
        to_white = not to_white
        cv2.imshow(WIN, white if to_white else black)
        cv2.waitKey(1)                      # this is where the window actually redraws
        t0 = time.perf_counter()
        flashes.append((t0, to_white, None, 0.0))
        if i > 0:
            report(i - 1, t0)
        running = hold(random.uniform(*GAP))
    if flashes:
        report(len(flashes) - 1, time.perf_counter())

    device.removeVideoListener()
    cv2.destroyAllWindows()

# Summary
delays = np.array([f[2] for f in flashes if f[2] is not None]) * 1000
print(f"\ndetected {len(delays)} of {len(flashes)} switches")
if len(delays):
    print(f"delay ms: median {np.median(delays):.0f}  min {delays.min():.0f}  max {delays.max():.0f}"
          f"  p10 {np.percentile(delays, 10):.0f}  p90 {np.percentile(delays, 90):.0f}")

# Save raw data for re-analysis
os.makedirs(OUT_DIR, exist_ok=True)
stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
base = os.path.abspath(os.path.join(OUT_DIR, f"latency_{stamp}" + ("_tel" if WITH_TELEMETRY else "")))
with open(base + "_frames.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_perf", "brightness"])
    w.writerows((f"{t:.4f}", f"{b:.1f}") for t, b in samples)
with open(base + "_flashes.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_perf", "to_white", "delay_ms", "contrast"])
    w.writerows((f"{t:.4f}", int(tw), "" if d is None else f"{d * 1000:.1f}", f"{c:.1f}")
                for t, tw, d, c in flashes)
print(f"  {base}_frames.csv\n  {base}_flashes.csv")
