"""
Where the outside tools are: adb and the scrcpy server. They are not part of the repo; download them into bin/
(see the README). adb is also taken from the PATH.
"""
import os
import shutil

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
BIN = os.path.join(ROOT, "bin")
RECORDINGS = os.path.join(ROOT, "recordings")
SCRCPY_VERSION = "3.3.1"                                # the server must match this version exactly
PHONE_JAR = f"/data/local/tmp/scrcpy-server-v{SCRCPY_VERSION}.jar"


def adb():
    """Path of adb: bin/ first, then the PATH."""
    for name in ("adb.exe", "adb"):
        p = os.path.join(BIN, name)
        if os.path.exists(p):
            return p
    p = shutil.which("adb")
    if p:
        return p
    raise FileNotFoundError(f"adb not found: put it in {BIN} or on the PATH (see the README)")


def scrcpy_server():
    """Path of the scrcpy server file on the PC (the scrcpy release names it scrcpy-server)."""
    for name in (f"scrcpy-server-v{SCRCPY_VERSION}", f"scrcpy-server-v{SCRCPY_VERSION}.jar", "scrcpy-server"):
        p = os.path.join(BIN, name)
        if os.path.exists(p):
            return p
    raise FileNotFoundError(f"scrcpy server v{SCRCPY_VERSION} not found in {BIN} (see the README)")
