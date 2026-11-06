"""
Live drone telemetry from DJI Fly's flight log.

Runs telemetry_js/live_log.mjs (Node.js) in the background. It reads the log off
the phone over adb, decrypts it, and sends one JSON row per log entry (10 Hz).
This class collects those rows on a thread.

    from neo_pilot.telemetry import LiveTelemetry
    with LiveTelemetry() as tel:
        row = tel.latest()          # newest row (dict) or None
        rows = tel.rows_since(n)    # all rows after the first n

Each row has t_pc (PC arrival time, unix s), dateTime (phone time, UTC string),
lat, lon, height, vpsHeight, vN, vE, vD (m/s, North/East/Down), roll, pitch,
yaw, gimbalPitch, gimbalYaw, gpsNum, isGPSUsed, flycState, isMotorOn, battery.
"""
import json
import os
import subprocess
import threading

from .paths import adb

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPT = os.path.join(HERE, "telemetry_js", "live_log.mjs")


def api_key():
    """DJI_API_KEY from the environment, or from the Windows user settings if this
    terminal was opened before the variable was set."""
    key = os.environ.get("DJI_API_KEY")
    if not key and os.name == "nt":
        import winreg
        try:
            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, "Environment") as k:
                key = winreg.QueryValueEx(k, "DJI_API_KEY")[0]
        except OSError:
            pass
    if not key:
        raise RuntimeError("DJI_API_KEY is not set (see the README: it decrypts the flight log)")
    return key


class LiveTelemetry:
    def __init__(self, existing=False, replay=None, on_event=print):
        """existing: follow the log already on the phone instead of waiting for a new one.
        replay: a 05_log_growth snapshots folder, to test without a phone.
        on_event: called with each status event (clock, new_log, keys)."""
        args = ["node", SCRIPT]
        if existing:
            args.append("--existing")
        if replay:
            args += ["--replay", replay]
        env = dict(os.environ, DJI_API_KEY=api_key(), NEO_PILOT_ADB=adb())
        self._proc = subprocess.Popen(args, stdout=subprocess.PIPE, text=True, env=env)
        self._rows = []
        self._lock = threading.Lock()
        self._on_event = on_event
        self.events = []
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        for line in self._proc.stdout:
            msg = json.loads(line)
            if "event" in msg:
                self.events.append(msg)
                if self._on_event:
                    self._on_event(msg)
                if msg["event"] == "new_log":
                    with self._lock:
                        self._rows = []
            else:
                with self._lock:
                    self._rows.append(msg)

    def latest(self):
        with self._lock:
            return self._rows[-1] if self._rows else None

    def rows_since(self, n):
        with self._lock:
            return self._rows[n:]

    def count(self):
        with self._lock:
            return len(self._rows)

    def alive(self):
        return self._proc.poll() is None

    def close(self):
        self._proc.kill()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
