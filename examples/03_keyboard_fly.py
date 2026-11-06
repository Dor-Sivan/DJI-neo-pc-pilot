"""
Fly the drone from the PC keyboard, in m/s and deg/s, through the safety layer.

Each key press changes a set speed by one step; it holds until changed (like a
cruise control), so several axes can run at once:
  w / s     forward / back           a / d     left / right        (drone frame)
  j / l     turn left / right        i / k     up / down
  space     STOP: all set speeds to 0 (the drone brakes and hovers)
  Esc       stop and quit
Any other key also stops.

The set speeds go through SafeDrone (the profile's caps and height band) and the
stick maps (drone/neo_maps.py). The Monitor stops everything when telemetry or
video stalls, the drone leaves the fence around where it was when you started
moving, or the sticks get no answer (a DJI pop-up). If this program dies or
hangs, the touch watchdog lets go of the sticks.

Take off by hand in DJI Fly first (landscape camera view, see the README).

Run:  python examples/03_keyboard_fly.py --env indoor
      python examples/03_keyboard_fly.py --env outdoor
      python examples/03_keyboard_fly.py --env indoor --sim      (simulated drone, no phone)
"""
import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from neo_pilot.flight.profiles import PROFILES          # noqa: E402
from neo_pilot.flight.safety import Monitor, SafeDrone  # noqa: E402

SHOW_SCALE = 0.5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", choices=["indoor", "outdoor"], required=True, help="flight profile: the limits")
    ap.add_argument("--sim", action="store_true", help="simulated drone instead of the phone")
    args = ap.parse_args()
    p = PROFILES[args.env]

    if args.sim:
        from neo_pilot.drone.sim import SimDrone
        drone = SimDrone(env=args.env)
    else:
        from neo_pilot.drone.neo import NeoDrone
        drone = NeoDrone()
    safe = SafeDrone(drone, p)
    monitor = Monitor(safe)

    # one step per key press; the smallest step is above the sticks' dead zone
    h_step = max(0.2, p.max_speed / 4)
    yaw_step, climb_step = 10.0, 0.1
    keys = {"w": ("forward", h_step), "s": ("forward", -h_step), "d": ("right", h_step), "a": ("right", -h_step),
            "l": ("yaw_rate", yaw_step), "j": ("yaw_rate", -yaw_step), "i": ("climb", climb_step),
            "k": ("climb", -climb_step)}
    limit = {"forward": p.max_speed, "right": p.max_speed, "yaw_rate": p.max_yaw_rate, "climb": p.max_climb}
    want = dict.fromkeys(limit, 0.0)
    note = ""

    print(f"profile {p.name}: speed <= {p.max_speed} m/s, climb <= {p.max_climb} m/s, turn <= {p.max_yaw_rate} "
          f"deg/s, height {p.h_min}-{p.h_max} m, fence {p.fence} m")
    print(__doc__.split("Each key")[1].split("The set")[0])
    win = "keyboard fly - focus this window; space STOP, Esc quit"
    cv2.namedWindow(win)
    try:
        if not drone.wait_ready(30.0):
            print("no video or telemetry yet: is the drone flying and DJI Fly on the camera view?")
        while drone.alive():
            moving = any(want.values())
            if moving:
                why = monitor.check(time.time())
                if why:
                    print(f"SAFETY STOP: {why}")
                    note = "SAFETY STOP: " + why
                    want = dict.fromkeys(limit, 0.0)
                    safe.release()
                else:
                    safe.rates(**want)
            else:
                safe.release()

            f = drone.latest_frame()
            if f is not None:
                img = cv2.resize(f.image, None, fx=SHOW_SCALE, fy=SHOW_SCALE)
                s = drone.latest_state()
                lines = ["set: fwd {forward:+.2f} right {right:+.2f} m/s  turn {yaw_rate:+.0f} deg/s  "
                         "up {climb:+.2f} m/s".format(**want),
                         "sticks: yaw {:+.2f} up {:+.2f} right {:+.2f} fwd {:+.2f}".format(*safe.command())]
                if s is not None:
                    lines.append(f"h {s.height:.1f} m  yaw {s.yaw:+.0f}  battery {s.battery:.0f}%  "
                                 f"telemetry {time.time() - s.t:.1f} s old")
                if note:
                    lines.append(note)
                cv2.rectangle(img, (5, 5), (img.shape[1] - 5, 15 + 22 * len(lines)), (0, 0, 0), -1)
                for k, text in enumerate(lines):
                    cv2.putText(img, text, (12, 27 + 22 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255) if moving else (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow(win, img)

            key = cv2.waitKey(20) & 0xFF
            if key == 255:
                continue
            ch = chr(key)
            if ch in keys:
                axis, step = keys[ch]
                if not moving:
                    monitor.start()             # the fence is around where the drone starts moving
                    note = ""
                want[axis] = float(np.clip(round(want[axis] + step, 3), -limit[axis], limit[axis]))
                continue
            want = dict.fromkeys(limit, 0.0)    # space or any other key: stop
            safe.release()
            if key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        safe.release()
        cv2.destroyAllWindows()
        drone.close()


if __name__ == "__main__":
    main()
