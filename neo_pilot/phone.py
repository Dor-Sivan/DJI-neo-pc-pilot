"""
Phone link: the phone's screen as video and injected touches, through scrcpy
(scrcpy.py, from DJI-Neo-to-PC), without its start-up race.

    from neo_pilot.phone import connect
    device = connect()
    frame = device.getLastFrame()
    ...
    device.destroy()
"""
import random
import socket
import subprocess
import time

from . import scrcpy
from .paths import PHONE_JAR, SCRCPY_VERSION, adb, scrcpy_server

PORT = 33333


class Phone(scrcpy.scrcpy):
    """
    scrcpy.scrcpy with a reliable start. The original waits a fixed 1 s for the
    phone server and connects once; if the server is not listening yet, adb
    accepts the connection and closes it at once, and the video thread dies.

    Here the server is started once and the video socket is reconnected until
    the server answers. With send_dummy_byte=true the server writes one 0x00
    byte on the first socket when it really accepted it (raw_stream=true turns
    that byte off, so it is switched back on after it).
    """

    def __init__(self, server_wait=10.0):
        self._deviceID = None
        self._SCID = f"{random.getrandbits(31):08x}"
        self._server_filename = PHONE_JAR
        self._adb_proc = None
        self._video_background_process = None
        self._socket_video = socket.socket()        # replaced below; destroy() closes whatever is here
        self._socket_control = socket.socket()

        # adb gets an empty stdin: inherited, it swallows what the caller reads (the touch watchdog's pipe)
        ADB = adb()
        subprocess.run([ADB, "forward", f"tcp:{PORT}", f"localabstract:scrcpy_{self._SCID}"], check=True,
                       stdin=subprocess.DEVNULL)
        subprocess.run([ADB, "push", scrcpy_server(), PHONE_JAR],
                       check=True, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
        self._adb_proc = subprocess.Popen([
            ADB, "shell", f"CLASSPATH={PHONE_JAR}",
            "app_process", "/", "com.genymobile.scrcpy.Server", SCRCPY_VERSION, f"scid={self._SCID}",
            "tunnel_forward=true", "video=true", "audio=false", "control=true", "cleanup=false",
            "raw_stream=true", "send_dummy_byte=true", "stay_awake=true"], stdin=subprocess.DEVNULL)

        t_start = time.perf_counter()
        deadline = t_start + server_wait
        tries = 0
        while True:
            if self._adb_proc.poll() is not None:
                self.destroy()
                raise RuntimeError("scrcpy server exited on the phone (see its messages above)")
            if time.perf_counter() > deadline:
                self.destroy()
                raise RuntimeError(f"scrcpy server did not answer within {server_wait} s")
            tries += 1
            s = socket.create_connection(("localhost", PORT))
            s.settimeout(1.0)
            try:
                first = s.recv(1)
            except socket.timeout:
                first = b""
            if first == b"\x00":
                break
            s.close()               # adb accepted for a server that isn't listening yet
            time.sleep(0.1)
        s.settimeout(None)
        self._socket_video = s
        self._socket_control = socket.create_connection(("localhost", PORT))
        print(f"phone server ready after {time.perf_counter() - t_start:.1f} s ({tries} connect tries)")
        self._video_background_process = scrcpy.BackgroundVideoCodec(self._socket_video)


def connect(max_attempts=2, wait_first_frame=5.0):
    """Start the phone link and wait for the first video frame."""
    for attempt in range(1, max_attempts + 1):
        try:
            device = Phone()
        except Exception as e:
            print(f"attempt {attempt}: {e}")
            continue
        deadline = time.perf_counter() + wait_first_frame
        while time.perf_counter() < deadline and video_alive(device):
            if device.getLastFrame() is not None:
                return device
            time.sleep(0.05)
        reason = "video thread died" if not video_alive(device) else "no frame in time"
        print(f"attempt {attempt}: {reason}, retrying")
        device.destroy()
    raise RuntimeError(f"no video after {max_attempts} attempts")


def video_alive(device):
    return device._video_background_process._thread.is_alive()
