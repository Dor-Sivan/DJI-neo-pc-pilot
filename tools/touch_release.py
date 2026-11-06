"""
ON THE GROUND, no drone needed: how to free a touch that a dead
PC program left pressed on the phone.

A flight test showed it: the program died while holding the yaw stick, and
the stick stayed pressed (the drone kept spinning). Android keeps an injected
touch down until someone sends its "up". Here we find what can send it.

Set up the phone once: Settings > Developer options > "Pointer location" ON.
The phone then draws every touch, and the top bar shows "P: 1 / 1" while one
is pressed. Open any app (or the home screen) in landscape.

  python tools/touch_release.py hold
        presses at the screen centre, moves a little, then the program dies
        without letting go. Check the phone: the touch should stay (P: 1).
  python tools/touch_release.py release <method>
        tries to free it. After each, check whether P went back to 0:
          same-id   new phone link, "up" for the dead program's pointer
                    (its id and position are saved by hold)
          tap       new phone link, a fresh press + up at the same place
          home      phone's Home key (adb), the app loses the touch
Run hold, then one method; repeat hold for the next method.
Result (2026-10-31): same-id and tap free it, home does not.

  python tools/touch_release.py guarded
        like hold, but with the watchdog (drone/touch_guard.py) running: it
        should free the touch by itself a few seconds after the program dies.
"""
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from neo_pilot.paths import RECORDINGS, adb             # noqa: E402
from neo_pilot.phone import connect                     # noqa: E402

SAVED = os.path.join(RECORDINGS, "held_touch.json")


def hold():
    device = connect()
    h, w = device.getLastFrame().shape[:2]
    pointer, x, y = 123456789, w // 2, h // 2
    device.touchDown(pointer, x, y)
    for dx in range(0, 60, 10):
        device.touchMove(pointer, x + dx, y)
        time.sleep(0.05)
    os.makedirs(RECORDINGS, exist_ok=True)
    with open(SAVED, "w") as f:
        json.dump({"pointer": pointer, "x": x + 50, "y": y, "screen": [w, h]}, f)
    print(f"holding pointer {pointer} at ({x + 50}, {y}) on a {w}x{h} screen; dying now without letting go")
    print("check the phone: Pointer location should still show the touch (P: 1)")
    sys.stdout.flush()
    time.sleep(0.5)
    os._exit(1)


def release(method):
    with open(SAVED) as f:
        held = json.load(f)
    if method == "home":
        subprocess.run([adb(), "shell", "input", "keyevent", "KEYCODE_HOME"], check=True)
    else:
        device = connect()
        if method == "same-id":
            device.touchUp(held["pointer"], held["x"], held["y"])
        elif method == "tap":
            device.touchDown(987654321, held["x"], held["y"])
            time.sleep(0.05)
            device.touchUp(987654321, held["x"], held["y"])
        else:
            sys.exit(f"unknown method {method}")
        time.sleep(0.5)
        device.destroy()
    print(f"{method}: sent. Check the phone: is the touch gone (P: 0)?")


def guarded():
    from neo_pilot.drone.touch_guard import TouchGuard
    device = connect()
    guard = TouchGuard()
    print("waiting 8 s for the watchdog's own link ('[touch guard] ready')")
    time.sleep(8.0)
    h, w = device.getLastFrame().shape[:2]
    pointer, x, y = 123456789, w // 2, h // 2
    device.touchDown(pointer, x, y)
    guard.pressed("L", pointer, x, y, w, h)
    print(f"holding pointer {pointer} at ({x}, {y}) with the watchdog on; dying in 2 s without letting go")
    print(f"check the phone: P: 1, then back to 0 a few seconds after the death (time it)")
    sys.stdout.flush()
    time.sleep(2.0)
    print(f"dying at {time.strftime('%H:%M:%S')}")
    sys.stdout.flush()
    os._exit(1)


if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "hold":
        hold()
    elif len(sys.argv) >= 2 and sys.argv[1] == "guarded":
        guarded()
    elif len(sys.argv) == 3 and sys.argv[1] == "release":
        release(sys.argv[2])
    else:
        sys.exit(__doc__)
