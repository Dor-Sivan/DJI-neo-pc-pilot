# bin/

Put **adb** and the **scrcpy server v3.3.1** here. They are not in the repo. The server must be exactly v3.3.1,
the version the code talks to.

**Windows**
1. Download `scrcpy-win64-v3.3.1.zip` from the
   [scrcpy v3.3.1 release](https://github.com/Genymobile/scrcpy/releases/tag/v3.3.1).
2. Copy these four files from it into this folder:
   ```
   adb.exe
   AdbWinApi.dll
   AdbWinUsbApi.dll
   scrcpy-server
   ```

**Linux and macOS**
1. Install adb: the Android [platform-tools](https://developer.android.com/tools/releases/platform-tools), or your
   package manager. It can stay on the PATH; it doesn't need to be in this folder.
2. Download the file `scrcpy-server-v3.3.1` from the same release into this folder.

**Check:** with the phone connected by USB, `adb devices` (or `bin/adb devices` on Windows) should list the phone
as `device`.
