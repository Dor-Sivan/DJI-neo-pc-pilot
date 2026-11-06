"""
The DJI Neo's measured numbers that do not need the phone link: video delay and stick maps.
Used by drone/neo.py and drone/sim.py.
"""

VIDEO_DELAY = 0.566             # s, frame vs telemetry: the flight suite's turns test o (2026-10-31, image motion vs yaw rate,
                                # +0.15 s on the flash test's 0.416 s hovering; the telemetry times are early)

# Stick -> response as (stick, response) points, in real DJI stick units (-1..1 = full deflection; the flight log's
# RC values confirm them since STICK_RADIUS = 108 px). Measured 2026-10-31 indoors and outdoors (GPS), +/- averaged,
# most likely in DJI Fly's CINE mode (climb/descent topped at 0.5 m/s, horizontal ~1.5-2 m/s): re-measure after
# switching modes. The first point is the dead-zone edge: below it the drone does nothing. Sticks are never sent
# inside the dead zone, so the response the code expects is one the drone really gives (the follow predictor relies
# on it). The last points are full stick (the flight suite's outdoor tables s, d, v: speed after a 3 s pulse, still rising).
YAW_MAP = [(0.20, 6.0), (0.30, 16.5), (0.40, 22.6), (0.50, 25.6), (0.60, 31.1), (0.80, 42.0), (1.00, 47.0)]   # deg/s
UP_MAP = [(0.40, 0.10), (0.60, 0.18), (0.80, 0.26), (1.00, 0.46)]       # m/s (telemetry steps 0.1 m/s)
SPEED_MAP = [(0.30, 0.20), (0.40, 0.28), (0.60, 0.53), (0.80, 0.79), (1.00, 1.30)]   # m/s, forward and right alike
