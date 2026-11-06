"""
Live drone telemetry on the PC (no video, no control).

Starts the live log reader (neo_pilot.telemetry.LiveTelemetry), prints a status line
once per second and saves every row to recordings/telemetry_<date>_<time>.csv.

The status line shows the newest row and its age: PC time now minus the row's
phone time, corrected by the phone/PC clock offset measured at start. Rows
whose dateTime is 1970 (it happens in some logs) have no age.

Run:   python examples/01_telemetry.py              start before connecting the drone (waits for a new log)
       python examples/01_telemetry.py --existing   follow the log already on the phone
Stop:  Ctrl+C.
"""
import csv
import os
import sys
import time
from datetime import datetime

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from neo_pilot.paths import RECORDINGS as OUT_DIR       # noqa: E402
from neo_pilot.telemetry import LiveTelemetry           # noqa: E402

FIELDS = ["t_pc", "dateTime", "flyTime", "lat", "lon", "height", "vpsHeight", "vN", "vE", "vD",
          "roll", "pitch", "yaw", "gimbalPitch", "gimbalYaw", "gpsNum", "isGPSUsed", "flycState",
          "isMotorOn", "battery"]


def phone_time(row):
    if row["dateTime"].startswith("1970"):
        return None
    return datetime.fromisoformat(row["dateTime"].replace("Z", "+00:00")).timestamp()


def main():
    replay = sys.argv[sys.argv.index("--replay") + 1] if "--replay" in sys.argv else None
    os.makedirs(OUT_DIR, exist_ok=True)
    out_path = os.path.join(OUT_DIR, "telemetry_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv")

    clock = {"offset": 0.0}

    def on_event(ev):
        if ev["event"] == "clock":
            clock["offset"] = ev["phone_minus_pc"]
        print(f"[{ev['event']}] " + ", ".join(f"{k}={v}" for k, v in ev.items() if k != "event"))

    saved = 0
    with LiveTelemetry(existing="--existing" in sys.argv, replay=replay, on_event=on_event) as tel, \
         open(out_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        print(f"saving to {out_path}\nCtrl+C to stop")
        try:
            while tel.alive() or saved < tel.count():
                time.sleep(1.0)
                new = tel.rows_since(saved)
                w.writerows(new)
                f.flush()
                saved += len(new)
                r = tel.latest()
                if r is None:
                    print("waiting for telemetry...")
                    continue
                pt = phone_time(r)
                age = f"{time.time() - (pt - clock['offset']):.2f} s" if pt else "n/a"
                print(f"{saved:5d} rows (+{len(new):2d})  age {age:>7}  h {r['height']:5.1f} m  "
                      f"vN {r['vN']:+5.1f} vE {r['vE']:+5.1f} vD {r['vD']:+5.1f}  yaw {r['yaw']:7.1f}  "
                      f"gps {r['gpsNum']:2d}  {r['flycState']}")
        except KeyboardInterrupt:
            pass
    print(f"saved {saved} rows to {out_path}")


if __name__ == "__main__":
    main()
