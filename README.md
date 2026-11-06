# DJI-neo-pc-pilot

Fly a **DJI Neo** from a PC program, in closed loop, through the phone that runs DJI Fly.

The Neo has no SDK: the only way to control it is the DJI Fly app on a phone. This project connects the phone to a
PC with a USB cable and gives a Python program:

- **video**: the camera picture from the phone screen, with an estimated capture time for every frame;
- **telemetry**, 10 rows per second: position (GPS), velocity, attitude, gimbal, height, battery, read live from DJI Fly's own flight log;
- **control in physical units**: "turn at 20 deg/s, fly forward at 1 m/s", turned into DJI Fly's on-screen joysticks with stick maps measured in flight;
- **safety**:
  - speed, height and distance limits;
  - automatic stops when the video or the telemetry stalls, or the drone stops obeying the sticks (for example, a DJI pop-up took the screen);
  - a watchdog that lets go of the sticks if your program crashes or hangs;
- **a simulator** that behaves like the measured Neo (delays, dead zones, wind), so you can try a program on the PC first;
- **examples**: keyboard flying, live capture, and a suite of flight tests (speed tables, a square on GPS, following a coloured target, a GPS waypoint route).

It builds on [DJI-Neo-to-PC](https://github.com/Penkov-D/DJI-Neo-to-PC), which drives the phone with
[scrcpy](https://github.com/Genymobile/scrcpy).

> **Fly safely.** This is experimental software that flies a real drone by simulating touches on a phone. Fly in
> open space, away from people, within the law where you fly. Always be ready to take over on the phone. Rehearse
> in the simulator first. You use it at your own risk.

## How it works

```
 drone ──radio──► phone (DJI Fly) ──USB──► PC
                    │  screen ──────── scrcpy video ─────► Frame (image, capture time)
                    │  flight log ──── adb + decryption ──► State (position, velocity, attitude, ...)
                    └◄─ touches on the virtual joysticks ◄── sticks / rates / velocity commands
```

| | DJI-Neo-to-PC | DJI-neo-pc-pilot |
|---|---|---|
| Control | raw stick values | also deg/s and m/s, in the drone frame or the world frame (east/north/up), through measured stick maps |
| Commands | a touch holds until the next call | re-sent 20 times a second; a command lasts 0.3 s unless renewed, then the sticks are released and the drone brakes |
| Program crash or hang | the stick stays held | a separate watchdog process with its own phone link lets go of the sticks |
| Telemetry | none | DJI Fly's flight log, read live and decrypted, 10 Hz |
| Timing | none | every frame and state gets a capture-time estimate on the PC clock (measured delays; the log's own clock fixes skipped rows) |
| Screen | portrait screen, positions from a settings file per phone | landscape full-screen camera view; the camera area and the side of the navigation bar are found automatically |
| Start-up | can lose the race with the phone server | waits for the server, retries |
| Safety | none | limits per flight profile, stops for stale data, fence, sticks with no answer |
| Simulator | no | yes, plus replay of recordings |

The program sees any drone through the small interface in [neo_pilot/drone/base.py](neo_pilot/drone/base.py):
`Frame`, `State`, `sticks()`, `rates()`, `velocity_enu()`. The Neo, the simulator and the replay are three
implementations of it, so a program written for one runs on the others.

## What you need

**Hardware**
- DJI Neo.
- An Android phone that runs DJI Fly and is connected to the Neo. Tested with a Xiaomi Redmi 23053RN02Y (Android 15).
- A USB cable from the phone to the PC.

**PC**
- Tested on Windows 11. Linux and macOS should work, but are not tested.
- Python 3.10 or newer: `pip install -r requirements.txt`.
- Node.js 18 or newer (tested with 24), for the telemetry decoder:
  ```
  cd neo_pilot/telemetry_js
  npm install
  ```
- **adb and the scrcpy server v3.3.1** (not included): see [bin/README.md](bin/README.md).

**Phone**
- **Developer options:** Settings → About phone → tap "Build number" 7 times.
- **USB debugging** on, in Developer options. Accept the "Allow USB debugging?" prompt the first time you connect.
- **Xiaomi phones only:** also turn on "USB debugging (Security settings)", or the touches are blocked.
- **Check the link:** `bin/adb devices` should list the phone as `device`.

**A DJI API key** (free). DJI Fly encrypts its flight logs, and the key is what lets the decoder
([dji-log-parser](https://github.com/lvauvillier/dji-log-parser)) fetch the decryption keys from DJI.
1. Sign in at [developer.dji.com](https://developer.dji.com), open the developer center, and **Create App**.
2. Choose the app type **Open API**, fill in the details, and activate the app from the email DJI sends.
3. The app's **App Key** is your API key.
4. Save it as the environment variable `DJI_API_KEY`.
   - Windows:
     ```
     setx DJI_API_KEY "your-key"
     ```
     The program also reads it from the user settings, so a terminal that was already open works too.
   - Linux and macOS: put `export DJI_API_KEY=your-key` in your shell profile.

   Keep the key private and don't commit it.

The PC needs internet when a flight log starts, so the decoder can fetch that log's keys. That takes about a second,
once per log.

## DJI Fly setup

1. Connect the Neo to DJI Fly as usual.
2. Choose the **manual (joystick) control**: the camera view shows two virtual joysticks. Keep the default stick
   mode (left = turn and up/down, right = forward/back and left/right).
3. **Turn the phone to landscape** and stay on the **full-screen camera view**. Either side up works.
   - The virtual joysticks only show while flying.
   - Only this screen is controlled: the portrait screen or a menu gets no sticks.
4. **Take off by hand** in DJI Fly and hover. Taking off and landing from the PC are not supported yet.
5. Close any pop-up DJI Fly shows. A pop-up takes the touches: the sticks then do nothing, and the safety monitor
   stops the test.

The telemetry only exists while the motors run. DJI Fly writes its flight log only then, so a program on the ground
sees video but no states.

## Running it

Rehearse first in the simulator. No phone or drone is needed; there is a simulated tennis ball to follow:
```
python examples/04_flight_suite.py --env indoor --sim still
python examples/03_keyboard_fly.py --env indoor --sim
```

With the drone, after taking off by hand:

| Example | What it does |
|---|---|
| `python examples/01_telemetry.py --existing` | prints the live telemetry once per second and saves every row (no video, no control) |
| `python examples/02_live_capture.py` | shows the camera view with telemetry and records video + telemetry with times (no control). Replay it with `neo_pilot.drone.replay.ReplayDrone` |
| `python examples/03_keyboard_fly.py --env indoor` | fly from the keyboard in m/s and deg/s through the safety layer: `w`/`s` forward/back, `a`/`d` left/right, `j`/`l` turn, `i`/`k` up/down, **space stops**, Esc quits |
| `python examples/04_flight_suite.py --env indoor` | flight tests, one key each (listed when it starts): watchdog hang test, square, follow and approach a coloured target (click it in the window), speed tables, turns, GPS route (`--env route`, waypoints in [examples/example_route.csv](examples/example_route.csv)). Each run saves its data in `recordings/` and prints a report |

In every program, **space** (or any key that isn't a test key) releases the sticks at once, and the drone brakes and
hovers. Taking over on the phone works at any time too.

The `--env` flight profile sets the limits. They are defined in
[neo_pilot/flight/profiles.py](neo_pilot/flight/profiles.py):

| Profile | Stick cap | Speed | Climb | Turn | Height | Fence |
|---|---|---|---|---|---|---|
| `indoor` | 0.6 | 0.4 m/s | 0.2 m/s | 30 °/s | 0.5–2 m | 2.5 m |
| `outdoor` | 1.0 | 3 m/s | 0.5 m/s | 60 °/s | 1.5–15 m | 40 m, needs GPS |
| `route` | 1.0 | 3 m/s | 0.5 m/s | 60 °/s | 1.5–22 m | 50 m, needs GPS |

Height is measured above the takeoff point. The minimum height uses the downward sensor when it reads.

### Your own program

```python
import time

from neo_pilot.drone.neo import NeoDrone          # or: from neo_pilot.drone.sim import SimDrone
from neo_pilot.flight.profiles import PROFILES
from neo_pilot.flight.safety import Monitor, SafeDrone

with NeoDrone() as drone:
    drone.wait_ready()                            # first frame and first state
    safe = SafeDrone(drone, PROFILES["outdoor"])  # the limits on every command
    monitor = Monitor(safe)
    monitor.start()
    while True:
        frame, state = drone.latest_frame(), drone.latest_state()
        # frame.image (BGR), frame.t; state.t, state.pos_enu, state.vel_enu, state.yaw, state.height, ...
        if monitor.check(time.time()):            # a reason to stop
            safe.release()
            break
        safe.rates(yaw_rate=10.0, forward=0.5)    # deg/s, m/s; renew it every loop (it lasts 0.3 s)
```

**Timing.** All times are on the PC clock (`time.time()`).
- `frame.t` and `state.t` are when the picture or measurement was taken, not when it arrived.
- `drone.state_at(t)` interpolates the state at any time, for example at a frame's capture time.
- **The data is late:** frames by about 0.5 s and states by about 1–1.5 s. Steering on the newest state alone
  oscillates, so take into account the commands you sent since. `Tracker` and `Route.predicted_yaw` in
  [neo_pilot/flight/tests.py](neo_pilot/flight/tests.py) show how.

## Measured behaviour and limits

All measured with this setup. The stick maps are in [neo_pilot/drone/neo_maps.py](neo_pilot/drone/neo_maps.py).

| | |
|---|---|
| Video delay (camera → PC) | ~0.4 s hovering; 0.57 s against the telemetry (used) |
| Telemetry delay | ~1–1.5 s; rows arrive in batches about every 0.7 s, and the phone sometimes skips rows |
| Stick → drone starts to answer | ~0.3 s |
| Top horizontal speed | ~1.9 m/s (in the DJI Fly mode we flew; the stick map stops at 1.3 m/s after 3 s) |
| Turn rate, full stick | 37–47 °/s (outdoors less than indoors) |
| Climb, full stick | ~0.5 m/s |
| Dead zone | sticks below ~0.2–0.4 do nothing; the code never sends them |
| Distance | DJI stops the Neo ~50 m from the takeoff point |

**Known problems**
- **DJI pop-ups** take the touches and the sticks stop working. The monitor detects the missing answer and stops the test.
- **The flight log stops:** DJI Fly once stopped writing its flight log in mid-air, so the telemetry stops. The monitor stops the test; restart DJI Fly.
- **Wrong heading:** the drone's reported heading was seen ~37° off for a few seconds, then it jumped back.
- **Only one phone tested.** The positions of the camera area and joysticks are fractions of the screen and the
  joystick radius is in pixels (`CAMERA_X`, `CAMERA_Y`, `STICK_X`, `STICK_Y`, `STICK_RADIUS` in
  [neo_pilot/drone/neo.py](neo_pilot/drone/neo.py)). On another phone, check them: the flight log's `RC` values
  show the stick DJI received for the stick you sent.
- **Focal length:** follow and approach take angles from a nominal focal length, `--fx` (default 1600 px at
  1920 px wide, measured on the live view by the turns test `o`; the recorded video on the drone is wider, ~1240).
- **Overlays in the video:** DJI Fly draws joysticks, buttons and numbers over the camera picture. The colour detector ignores the image edges.
- **No takeoff or landing** from the PC, and no reconnect if the USB link drops during a flight.

## Tools

- [tools/video_latency.py](tools/video_latency.py): measures the camera → PC delay. The PC screen flashes, the drone's camera looks at it, and the program times the change in the video.
- [tools/touch_release.py](tools/touch_release.py): checks on the ground that a touch left pressed by a dead program is released, with and without the watchdog.

## Layout

```
neo_pilot/
  phone.py, scrcpy.py      phone link: screen video, touch injection
  telemetry.py             live telemetry (runs telemetry_js/live_log.mjs)
  paths.py                 where adb and the scrcpy server are
  drone/                   base.py (the interface), neo.py (the Neo), neo_maps.py (measured maps),
                           touch_guard.py (watchdog), sim.py (simulator), replay.py (recordings)
  flight/                  safety.py, profiles.py, tests.py (the flight tests), run.py (their files and
                           reports), detector.py (colour target)
examples/                  01-04 above, example_route.csv
tools/                     video_latency.py, touch_release.py
```

## Credits

- [DJI-Neo-to-PC](https://github.com/Penkov-D/DJI-Neo-to-PC) by Penkov-D: the original idea and the scrcpy client
  ([neo_pilot/scrcpy.py](neo_pilot/scrcpy.py)).
- [scrcpy](https://github.com/Genymobile/scrcpy) (Apache 2.0): the phone server that sends the screen and takes the touches.
- [dji-log-parser](https://github.com/lvauvillier/dji-log-parser) (MIT): decrypts and decodes DJI flight logs.

Not affiliated with or endorsed by DJI. DJI, DJI Fly and DJI Neo are trademarks of SZ DJI Technology Co., Ltd.

## License

MIT, see [LICENSE](LICENSE). By Dor Sivan.
