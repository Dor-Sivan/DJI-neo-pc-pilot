"""
Flight test suite: many tests per flight, one key each, with a flight profile
(indoor / outdoor / route limits) and a safety monitor.

Take off by hand in DJI Fly (landscape camera view), hover, focus the video
window, press a test's key. The start prints the plan for this environment and
every key (the tests are in neo_pilot/flight/tests.py).

Always:
  space   ABORT the running test (release; any key that is not a test key does the same)
  click   pick the target colour; follow and approach hover until it (a fresh click each run)
  q       quit

Safety (neo_pilot/flight/safety.py), while a test runs: profile limits on every command
(stick cap, speed/climb/turn caps, height band), and an abort when telemetry or
video stops, the drone leaves the fence around where the test started, or the
sticks get no answer (a DJI pop-up). The touch watchdog (drone/touch_guard.py)
releases the sticks if this program dies or hangs.

Every run writes recordings/flight_<date>_<time>_<env>_<test>_*.csv and prints
its report (with wind, heights, battery, GPS) ~3 s after it ends.

Run:  python examples/04_flight_suite.py --env indoor
      python examples/04_flight_suite.py --env outdoor --landmark 123.5
      python examples/04_flight_suite.py --env route                  (examples/example_route.csv, keys r and t)
      python examples/04_flight_suite.py --env route --route my_route.csv
      python examples/04_flight_suite.py --env indoor --sim still     (rehearse on the PC: still|sweep|walk)
"""
import argparse
import os
import sys
import time
from types import SimpleNamespace

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from neo_pilot.flight.detector import ColourDetector     # noqa: E402
from neo_pilot.flight.profiles import PROFILES           # noqa: E402
from neo_pilot.flight.run import AFTER, Run, common_report   # noqa: E402
from neo_pilot.flight.safety import Monitor, SafeDrone   # noqa: E402
from neo_pilot.flight.tests import PLAN, TESTS           # noqa: E402
from neo_pilot.paths import RECORDINGS                   # noqa: E402

OUT_DIR = RECORDINGS
SHOW_SCALE = 0.5


def flying(d, profile):
    s = d.latest_state()
    if s is None:
        return False, "no telemetry (motors off?)"
    age = time.time() - s.t_arrival
    if age > 3:
        return False, f"telemetry {age:.0f} s old"
    gps = f"GPS {s.raw.get('gpsNum', '?')} sats {'USED' if s.gps_ok else 'not used'}"
    ground = "" if s.ground is None else f" (ground {s.ground:.1f})"
    status = f"h {s.height:.1f} m{ground}  yaw {s.yaw:+.0f}  {s.mode}  {gps}  battery {s.battery:.0f}%"
    if not s.motors_on or (s.height < 0.3 and (s.ground or 0) < 0.3):
        return False, "not flying: " + status
    if profile.need_gps and not s.gps_ok:
        return False, "waiting for GPS: " + status
    return True, status


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--env", choices=list(PROFILES), required=True, help="flight profile: the limits")
    ap.add_argument("--sim", choices=["still", "sweep", "walk"], help="simulated drone with this target motion")
    ap.add_argument("--landmark", type=float, help="true compass bearing of the landmark (test l)")
    ap.add_argument("--route", default=os.path.join(HERE, "example_route.csv"), help="route CSV (keys r, t)")
    ap.add_argument("--ball-size", type=float, default=0.067, help="target diameter, m (tennis ball 0.067)")
    ap.add_argument("--fx", type=float, default=1600.0,
                    help="focal length of the live view, px at 1920 px image width (follow / approach angles and "
                         "range; the turns test o measures it)")
    args = ap.parse_args()
    args.route = os.path.abspath(args.route)
    profile = PROFILES[args.env]

    if args.sim:
        from neo_pilot.drone.sim import SimDrone
        drone = SimDrone(env=args.env, ball=args.sim, wind=(2.0, -1.0))
    else:
        from neo_pilot.drone.neo import NeoDrone
        drone = NeoDrone()
    safe = SafeDrone(drone, profile)
    detector = ColourDetector()
    ctx = SimpleNamespace(drone=drone, safe=safe, profile=profile, detector=detector, args=args, sim=bool(args.sim))
    tests = {cls.key: cls(ctx) for cls in TESTS}
    mine = [t for t in tests.values() if profile.name in t.envs]
    monitor = Monitor(safe)

    print(f"\nprofile {profile.name}: max stick {profile.max_stick}, speed {profile.max_speed} m/s, height "
          f"{profile.h_min}-{profile.h_max} m, fence {profile.fence} m, GPS {'needed' if profile.need_gps else 'not needed'}")
    print("plan:\n  " + "\n  ".join(PLAN[profile.name]))
    print("keys:\n  " + "\n  ".join(f"{t.key}  {t.title}" for t in mine) + "\n  space ABORT   click = pick colour   q quit\n")

    win = f"flight suite {profile.name}{' (SIM)' if args.sim else ''}  - keys in the terminal, space ABORT, q quit"
    shown = {"image": None}

    def on_click(event, x, y, *_):
        if event != cv2.EVENT_LBUTTONDOWN or shown["image"] is None:
            return
        waiting = active is not None and getattr(active, "waiting", False)
        if active is not None and not waiting:
            print("click ignored while a test runs (it would change the target colour)")
            return
        hsv = detector.pick(shown["image"], int(x / SHOW_SCALE), int(y / SHOW_SCALE))
        print(f"target colour HSV {hsv}" + ("   LOW SATURATION: grey/white, will also match walls - click the "
                                            "middle of a coloured target" if hsv[1] < 80 else ""))
        if waiting:
            active.clicked(time.time())

    active, run, pending, seq = None, None, [], None

    def stop(why):
        nonlocal active, run
        safe.release()
        if active is None:
            return
        run.event(why)
        run.t_end = time.time()
        pending.append((run.t_end + AFTER, run, active))
        active, run = None, None

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_click)
    if args.sim:                # rehearsal: pick the ball's colour automatically
        detector.hsv = __import__("numpy").array([35.0, 171.0, 241.0])
    try:
        while drone.alive():
            now = time.time()
            f = drone.latest_frame()
            if active is not None:
                why = monitor.check(now)
                if why:
                    print(f"\nSAFETY ABORT: {why}")
                    stop("SAFETY ABORT: " + why)
                else:
                    if f is not None and f.seq != seq:
                        active.on_frame(f, now)
                    if not active.step(now):
                        stop("done")
            if f is not None:
                seq = f.seq

            for item in [p for p in pending if now >= p[0]]:
                pending.remove(item)
                _, r, test = item
                states = drone.states()
                print(f"\n=== {r.name} report ===")
                report = common_report(r, states) + test.report(states)
                print("\n".join(report))
                r.save(states, report)

            if f is not None:
                shown["image"] = f.image
                img = cv2.resize(f.image, None, fx=SHOW_SCALE, fy=SHOW_SCALE)
                ok, status = flying(drone, profile)
                cmd = safe.command()
                lines = [status, f"RUNNING {active.name} (space = ABORT)" if active else
                         "keys: " + " ".join(t.key for t in mine) + "  (see terminal)",
                         "sticks: yaw {:+.2f} up {:+.2f} right {:+.2f} fwd {:+.2f}".format(*cmd)]
                if active:
                    lines += active.overlay()
                    det = active.marker()
                    if det is not None:
                        cv2.drawMarker(img, (int(det.u * SHOW_SCALE), int(det.v * SHOW_SCALE)), (0, 255, 0),
                                       cv2.MARKER_CROSS, 30, 2)
                        if det.box:
                            x, y, w, h = (int(a * SHOW_SCALE) for a in det.box)
                            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)
                if detector.hsv is None:
                    lines.append("click on the target to pick its colour")
                hh, ww = img.shape[:2]
                cv2.drawMarker(img, (ww // 2, hh // 2), (255, 255, 255), cv2.MARKER_TILTED_CROSS, 14, 1)
                cv2.rectangle(img, (5, 30), (ww - 5, 40 + 22 * len(lines)), (0, 0, 0), -1)
                for i, text in enumerate(lines):
                    cv2.putText(img, text, (12, 52 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 0, 255) if active else (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow(win, img)

            key = cv2.waitKey(5) & 0xFF
            if key == 255:
                continue
            ch = chr(key)
            if active is None and ch in tests:
                t = tests[ch]
                ok, status = flying(drone, profile)
                why = t.can_start() or (None if ok else status)
                if why:
                    print(f"{t.name}: not starting - {why}")
                    continue
                run = Run(OUT_DIR, profile.name, t.name)
                monitor.start()
                t.start(run)
                active = t
                print(f"\n{t.name}: started")
                continue
            if active is not None and ch == active.key:
                stop("stopped")
                continue
            stop("ABORT" if active else "release")
            if ch == "q":
                break
    except KeyboardInterrupt:
        pass
    finally:
        safe.release()
        if active is not None:
            stop("program closing")
        cv2.destroyAllWindows()
        if pending:
            time.sleep(max(0.0, max(p[0] for p in pending) - time.time()))
            states = drone.states()
            for _, r, test in pending:
                print(f"\n=== {r.name} report ===")
                report = common_report(r, states) + test.report(states)
                print("\n".join(report))
                r.save(states, report)
        drone.close()


if __name__ == "__main__":
    main()
