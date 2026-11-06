"""
Live frames + telemetry through the drone interface (neo_pilot/drone/), no control.

Uses only the drone-agnostic interface (Drone, Frame, State), so the same script
works with another drone's adapter. Here: NeoDrone (phone screen video + live
flight log). A recording plays back through neo_pilot.drone.replay.ReplayDrone.

Shows the camera view with the newest telemetry written on it, and saves into
recordings/:
  cap_<date>_<time>.mp4          camera area, every frame, scaled to SCALE
  cap_<date>_<time>_frames.csv   one row per frame in the .mp4:
      seq, t (capture estimate, PC unix s), t_arrival,
      roll, pitch, yaw, gimbal_pitch, gimbal_yaw, e, n, u
      -> telemetry interpolated to the frame's capture time (empty if none)
  cap_<date>_<time>_states.csv   every telemetry row: times, ENU position and
      velocity, attitude, GPS, mode, battery

Run:   python examples/02_live_capture.py           drone already connected to DJI Fly
       python examples/02_live_capture.py --new-log  start first, then connect the drone
Stop:  q in the video window, or Ctrl+C.
"""
import csv
import os
import queue
import sys
import time
from datetime import datetime

import cv2

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from neo_pilot.drone.neo import NeoDrone      # noqa: E402
from neo_pilot.paths import RECORDINGS as OUT_DIR   # noqa: E402

SCALE = 0.5
NOMINAL_FPS = 48        # only for video players; real times are in the .csv

frames = queue.Queue(maxsize=200)
dropped = 0


def on_frame(f):
    global dropped
    try:
        frames.put_nowait(f)
    except queue.Full:
        dropped += 1


def overlay(img, d, fps):
    s = d.latest_state()
    now = time.time()
    if s is None:
        lines = ["waiting for telemetry..."]
    else:
        pos = "no fix" if s.pos_enu is None else "E {:+.1f} N {:+.1f} U {:+.1f}".format(*s.pos_enu)
        lines = [f"state age {now - s.t:.2f} s   video fps {fps}",
                 f"h {s.height:.1f} m   {pos}",
                 "v E {:+.1f} N {:+.1f} U {:+.1f} m/s".format(*s.vel_enu),
                 f"yaw {s.yaw:+.1f}  pitch {s.pitch:+.1f}  roll {s.roll:+.1f}  gimbal {s.gimbal_pitch:+.1f}",
                 f"{s.mode}  gps {'OK' if s.gps_ok else 'no'} ({s.raw['gpsNum']})  battery {s.battery:.0f}%"]
    # white text on a dark box (display copy only; not written to the .mp4)
    cv2.rectangle(img, (5, 5), (5 + 560, 15 + 22 * len(lines)), (0, 0, 0), -1)
    for i, text in enumerate(lines):
        cv2.putText(img, text, (12, 27 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)


def save_csvs(d, base, frame_rows):
    states = d.states()
    with open(base + "_frames.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "t", "t_arrival", "roll", "pitch", "yaw", "gimbal_pitch", "gimbal_yaw", "e", "n", "u"])
        for seq, t, t_arr in frame_rows:
            s = d.state_at(t)
            att = ["" for _ in range(8)] if s is None else (
                [f"{s.roll:.2f}", f"{s.pitch:.2f}", f"{s.yaw:.2f}", f"{s.gimbal_pitch:.2f}", f"{s.gimbal_yaw:.2f}"]
                + (["", "", ""] if s.pos_enu is None else [f"{x:.3f}" for x in s.pos_enu]))
            w.writerow([seq, f"{t:.3f}", f"{t_arr:.3f}"] + att)
    with open(base + "_states.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["seq", "t", "t_arrival", "e", "n", "u", "vE", "vN", "vU", "height", "roll", "pitch", "yaw",
                    "gimbal_pitch", "gimbal_yaw", "lat", "lon", "gps_ok", "gps_num", "motors_on", "mode",
                    "battery", "flyTime", "dateTime"])
        for s in states:
            pos = ["", "", ""] if s.pos_enu is None else [f"{x:.3f}" for x in s.pos_enu]
            w.writerow([s.seq, f"{s.t:.3f}", f"{s.t_arrival:.3f}", *pos, *[f"{x:.2f}" for x in s.vel_enu],
                        f"{s.height:.2f}", f"{s.roll:.2f}", f"{s.pitch:.2f}", f"{s.yaw:.2f}",
                        f"{s.gimbal_pitch:.2f}", f"{s.gimbal_yaw:.2f}", s.lat or "", s.lon or "", int(s.gps_ok),
                        s.raw["gpsNum"], int(s.motors_on), s.mode, s.battery, s.raw["flyTime"], s.raw["dateTime"]])
    return len(states)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    base = os.path.join(OUT_DIR, "cap_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    frame_rows = []     # (seq, t, t_arrival) per frame written
    writer = size = None
    n_report, t_report, fps = 0, time.time(), 0

    with NeoDrone(existing_log="--new-log" not in sys.argv) as d:
        d.add_frame_listener(on_frame)
        print("press q in the video window to stop")
        try:
            while True:
                try:
                    f = frames.get(timeout=1.0)
                except queue.Empty:
                    if not d.alive():
                        print("link lost, stopping")
                        break
                    print("no frame for 1 s")
                    continue

                small = cv2.resize(f.image, None, fx=SCALE, fy=SCALE, interpolation=cv2.INTER_AREA)
                if writer is None:
                    size = (small.shape[1], small.shape[0])
                    writer = cv2.VideoWriter(base + ".mp4", cv2.VideoWriter_fourcc(*"mp4v"), NOMINAL_FPS, size)
                elif (small.shape[1], small.shape[0]) != size:
                    small = cv2.resize(small, size)
                writer.write(small)
                frame_rows.append((f.seq, f.t, f.t_arrival))

                if f.t_arrival - t_report >= 1.0:
                    fps, n_report, t_report = len(frame_rows) - n_report, len(frame_rows), f.t_arrival
                    s = d.latest_state()
                    print(f"frames {len(frame_rows):6d}  fps {fps:3d}  states {len(d.states()):5d}  "
                          f"state age {time.time() - s.t if s else float('nan'):.2f} s  dropped {dropped}")
                show = small.copy()
                overlay(show, d, fps)
                cv2.imshow("live capture (q to stop)", show)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        except KeyboardInterrupt:
            pass
        finally:
            if writer is not None:
                writer.release()
            cv2.destroyAllWindows()
            n_states = save_csvs(d, base, frame_rows)

    print(f"\nsaved {len(frame_rows)} frames and {n_states} telemetry rows, dropped {dropped} frames")
    print(f"  {base}.mp4\n  {base}_frames.csv\n  {base}_states.csv")


if __name__ == "__main__":
    main()
