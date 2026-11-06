"""
Replay adapter: plays a recording made by examples/02_live_capture.py back
through the Drone interface, at real speed, with the original delays.

Each frame is handed out when its original arrival time comes round again, and
each state only once its original arrival time has passed, so a consumer sees
the same timing (and the same telemetry lag) as during the flight. All times
are shifted to the current clock.

    from neo_pilot.drone.replay import ReplayDrone
    with ReplayDrone("recordings/cap_20261030_134931") as d: ...
"""
import threading
import time

import av
import numpy as np
import pandas as pd

from .base import Drone, Frame, State


class ReplayDrone(Drone):
    def __init__(self, base, video_delay=0.288):
        """base: recording path without extension (…/cap_<stamp>)."""
        super().__init__()
        self.video_delay = video_delay
        self._frames = pd.read_csv(base + "_frames.csv")
        self._rows = pd.read_csv(base + "_states.csv")
        self._shift = time.time() + 0.5 - self._frames.t_arrival.iloc[0]     # recording time -> now
        self._all_states = [self._state(r) for r in self._rows.itertuples()]
        self._arrivals = np.maximum.accumulate([s.t_arrival for s in self._all_states])
        self._path = base + ".mp4"
        self._done = False
        self._stop = False
        self._thread = threading.Thread(target=self._play, daemon=True)
        self._thread.start()

    def _state(self, r):
        num = lambda x: None if pd.isna(x) else float(x)
        pos = None if pd.isna(r.e) else np.array([r.e, r.n, r.u])
        return State(
            t=r.t + self._shift, t_arrival=r.t_arrival + self._shift, seq=int(r.seq), pos_enu=pos,
            vel_enu=np.array([r.vE, r.vN, r.vU]), height=r.height, roll=r.roll, pitch=r.pitch, yaw=r.yaw,
            gimbal_pitch=r.gimbal_pitch, gimbal_yaw=r.gimbal_yaw, lat=num(r.lat), lon=num(r.lon),
            gps_ok=bool(r.gps_ok), motors_on=bool(r.motors_on), mode=r.mode, battery=r.battery,
            raw={"gpsNum": r.gps_num, "flyTime": r.flyTime, "dateTime": r.dateTime})

    def _play(self):
        with av.open(self._path) as c:
            for k, fr in enumerate(c.decode(c.streams.video[0])):
                if self._stop or k >= len(self._frames):
                    break
                row = self._frames.iloc[k]
                t_arr = row.t_arrival + self._shift
                wait = t_arr - time.time()
                if wait > 0:
                    time.sleep(wait)
                self._new_frame(Frame(image=fr.to_ndarray(format="bgr24"), t=row.t + self._shift,
                                      t_arrival=t_arr, seq=int(row.seq)))
        self._done = True

    def states(self):
        n = int(np.searchsorted(self._arrivals, time.time(), side="right"))
        return self._all_states[:n]

    def alive(self):
        return not self._done

    def close(self):
        self._stop = True
