"""
Watchdog for injected touches: if the program holding a virtual stick dies,
let go of it.

Android keeps an injected touch pressed until its "up" arrives; a program that
dies can't send it, so the stick stays held (seen in a flight test: the drone
kept spinning). tools/touch_release.py showed what frees it: a phone link
sending "up" for the same pointer id (the phone's Home key does not).

TouchGuard (start it after the main phone link is up) starts this file as a separate process and tells it, over a pipe,
which touches are down. The watchdog opens its own control-only phone link
(a second scrcpy server: no video, own port) at start, so it is ready. When the
pipe closes without "bye" (the parent died: crash, os._exit, killed from the
task manager), it sends "up" for every touch still down at once. If its own
link is not up, it falls back to a full new phone link (~5 s).

A parent that hangs without dying (the pipe stays open) is caught by a
heartbeat: once the parent has sent one, more than HANG_AFTER without the next
one while a touch is held -> the touches are released the same way (the parent
may come back; it then finds its sticks already up).

    guard = TouchGuard()
    guard.pressed("L", pointer, x, y, screen_w, screen_h)  ...  guard.released("L")
    guard.close()                       # clean exit: the watchdog just quits
"""
import os
import queue
import random
import signal
import socket
import struct
import subprocess
import sys
import threading
import time

if __package__ in (None, ""):          # started as a file by TouchGuard: make the package importable
    sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")))
from neo_pilot.paths import PHONE_JAR, SCRCPY_VERSION, adb, scrcpy_server     # noqa: E402

ADB = adb()
PORT = 33334                    # the main phone link uses 33333
ACTION_UP = 1
HANG_AFTER = 1.0                # s without a heartbeat (once heartbeats started) -> release


class TouchGuard:
    def __init__(self):
        flags = subprocess.CREATE_NEW_PROCESS_GROUP if os.name == "nt" else 0    # Ctrl+C must not stop it
        self._proc = subprocess.Popen([sys.executable, os.path.abspath(__file__)], stdin=subprocess.PIPE,
                                      text=True, creationflags=flags)

        self._lock = threading.Lock()

    def _send(self, line):
        with self._lock:            # called from the main thread and the stick sender
            try:
                self._proc.stdin.write(line + "\n")
                self._proc.stdin.flush()
            except (OSError, ValueError):
                pass

    def heartbeat(self):
        self._send("hb")

    def pressed(self, key, pointer, x, y, screen_w, screen_h):
        self._send(f"down {key} {pointer} {x} {y} {screen_w} {screen_h}")

    def released(self, key):
        self._send(f"up {key}")

    def close(self):
        self._send("bye")
        try:
            self._proc.stdin.close()
        except OSError:
            pass


# ---------------- the watchdog process ----------------
def log(msg):
    print(f"[touch guard] {msg}", flush=True)


def control_link(timeout=15.0):
    """Own scrcpy server with control only; returns its socket once the server has answered."""
    scid = f"{random.getrandbits(31):08x}"
    # every adb gets its own empty stdin: it would otherwise inherit ours (the parent's pipe) and swallow its lines
    # ("down", "hb"): seen 2026-11-02, the hang check never fired
    quiet = dict(stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    run = lambda *a: subprocess.run([ADB, *a], check=True, **quiet)
    run("forward", f"tcp:{PORT}", f"localabstract:scrcpy_{scid}")
    # the main phone link has pushed the server already; pushing again while it runs from the file could break it
    have = subprocess.run([ADB, "shell", "ls", PHONE_JAR], stdin=subprocess.DEVNULL,
                          capture_output=True, text=True)
    if have.returncode != 0:
        run("push", scrcpy_server(), PHONE_JAR)
    server = subprocess.Popen([
        ADB, "shell", f"CLASSPATH={PHONE_JAR}", "app_process", "/", "com.genymobile.scrcpy.Server",
        SCRCPY_VERSION, f"scid={scid}", "tunnel_forward=true", "video=false", "audio=false", "control=true",
        "cleanup=false", "send_device_meta=false", "send_dummy_byte=true"], **quiet)
    deadline = time.time() + timeout
    while time.time() < deadline:
        if server.poll() is not None:
            raise RuntimeError("guard server exited on the phone")
        s = socket.create_connection(("localhost", PORT))
        s.settimeout(1.0)
        try:
            first = s.recv(1)
        except socket.timeout:
            first = b""
        if first == b"\x00":        # the server accepted this socket
            s.settimeout(None)
            return s
        s.close()
        time.sleep(0.2)
    raise RuntimeError("guard server did not answer")


def touch_up(sock, pointer, x, y, w, h):
    # scrcpy INJECT_TOUCH_EVENT: type, action, pointer id, x, y, screen w, h, pressure, action button, buttons
    sock.sendall(struct.pack(">BBQiiHHHII", 2, ACTION_UP, pointer, x, y, w, h, 0, 0, 0))


def release_all(down, link, why):
    """Send "up" for every held touch; own link first, a full phone link if that fails."""
    t0 = time.time()
    log(f"{why} with {len(down)} touch(es) held: releasing")
    if "sock" in link:
        try:
            for touch in down.values():
                touch_up(link["sock"], *touch)
            log(f"released after {time.time() - t0:.2f} s (own link)")
            return
        except OSError as e:
            log(f"own link broke ({e}); full phone link")
            link.pop("sock", None)
    from neo_pilot.phone import connect
    for attempt in range(3):
        try:
            device = connect()
            for pointer, x, y, _, _ in down.values():
                device.touchUp(pointer, x, y)
            time.sleep(0.3)
            device.destroy()
            log(f"released after {time.time() - t0:.1f} s (full phone link)")
            return
        except Exception as e:
            log(f"attempt {attempt + 1} failed: {e}")
    log("COULD NOT RELEASE - take over by hand")


def watch():
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    link = {}

    def connect_early():
        try:
            link["sock"] = control_link()
            log("ready")
        except Exception as e:
            log(f"own link failed ({e}); would fall back to a full phone link")

    early = threading.Thread(target=connect_early, daemon=True)
    early.start()

    lines = queue.Queue()

    def read():
        for line in sys.stdin:      # ends when the parent's pipe closes
            lines.put(line)
        lines.put(None)

    threading.Thread(target=read, daemon=True).start()

    down = {}                       # key -> (pointer, x, y, w, h)
    last_hb = None                  # time of the newest heartbeat; None until the parent sends one
    hung = False
    while True:
        try:
            line = lines.get(timeout=0.05)
        except queue.Empty:
            line = ""
        if down and last_hb is not None and not hung and time.time() - last_hb > HANG_AFTER:
            early.join(timeout=5.0)
            release_all(down, link, f"no heartbeat for {time.time() - last_hb:.1f} s (parent hung)")
            hung = True
        if line == "":
            continue
        if line is None:            # pipe closed without "bye": the parent died
            if down and not hung:
                early.join(timeout=5.0)
                release_all(down, link, "parent died")
                time.sleep(0.3)
            return
        p = line.split()
        if not p:
            continue
        if p[0] == "hb":
            if hung:
                log("parent alive again (its sticks were released)")
                hung = False
                down.clear()
            last_hb = time.time()
        elif p[0] == "down":
            down[p[1]] = tuple(int(v) for v in p[2:7])
        elif p[0] == "up":
            down.pop(p[1], None)
        elif p[0] == "bye":         # clean exit; still let go of anything we think is held (harmless if up)
            if down and not hung:
                early.join(timeout=5.0)
                release_all(down, link, "clean exit")
                time.sleep(0.3)
            return


if __name__ == "__main__":
    watch()
