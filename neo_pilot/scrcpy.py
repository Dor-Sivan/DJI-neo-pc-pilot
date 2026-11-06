# From DJI-Neo-to-PC (https://github.com/Penkov-D/DJI-Neo-to-PC), unchanged. neo_pilot/phone.py starts it reliably.
import subprocess
from threading import Thread, Lock, Event
from collections.abc import Callable
import socket
import av
import random
import time
import numpy


class scrcpy:
    """
    scrcpy - class that utilizes the open source program SCRCPY:
        https://github.com/Genymobile/scrcpy/tree/v3.3.1
    This class uses the original device server that scrcpy provides, and gives
    small functionality of it to the user, via code. This includes getting live
    video feed, and registering live touch events.
    """

    __ACTION_DOWN__ = 0x00      # MotionEvent.ACTION_DOWN
    __ACTION_UP__   = 0x01      # MotionEvent.ACTION_UP
    __ACTION_MOVE__ = 0x02      # MotionEvent.ACTION_MOVE

    __SCID_BITS__ = 31              # Number of bits for scrcpy's random ID
    __ADB_LOOPBACK_ADDR__ = "localhost"
    __ADB_LOOPBACK_PORT__ = 33333   # Port to connect over the ADB tunnel
    __SCRCPY_VERSION__ = "3.3.1"    # SCRCPY version in use


    def __init__(self, device : str | None = None):
        """
        Connect a device with SCRCPY server.

        Args:
            device (str | None): string representing the device ID (useful if
                several devices are connected). If None, and only one device is
                connected to the PC, it will use it as default.
        """

        self._deviceID = device
        self._SCID = f"{random.getrandbits(scrcpy.__SCID_BITS__):08x}"
        self._server_filename = f"scrcpy-server-v{scrcpy.__SCRCPY_VERSION__}.jar"

        self._adb_proc = None
        self._socket_video = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._socket_control = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._video_background_process = None


        # Launch ADB tunnel for device communication
        adb_forward_command = [
            "adb", "forward",
            f"tcp:{scrcpy.__ADB_LOOPBACK_PORT__}",
            f"localabstract:scrcpy_{self._SCID}"
        ]
        
        if self._deviceID:
            adb_forward_command.insert(1, "-s")
            adb_forward_command.insert(2, self._deviceID)
        
        subprocess.run(adb_forward_command, check = True)


        # Push the server to the device
        adb_push_command = [
            "adb", "push",
            self._server_filename,
            f"/data/local/tmp/{self._server_filename}"
        ]
        
        if self._deviceID:
            adb_push_command.insert(1, "-s")
            adb_push_command.insert(2, self._deviceID)
        
        subprocess.run(adb_push_command, check = True)


        # Launch the SCRCPY server on the device
        adb_shell_command = [
            "adb", "shell",
            f"CLASSPATH=/data/local/tmp/{self._server_filename}",
            "app_process", "/", "com.genymobile.scrcpy.Server",
            scrcpy.__SCRCPY_VERSION__,
            f"scid={self._SCID}",
            "tunnel_forward=true",
            "video=true",
            "audio=false",
            "control=true",
            "cleanup=false",
            "raw_stream=true",
            "stay_awake=true",
        ]
        
        if self._deviceID:
            adb_shell_command.insert(1, "-s")
            adb_shell_command.insert(2, self._deviceID)
        
        self._adb_proc = subprocess.Popen(adb_shell_command)
        time.sleep(1.0)     # Wait 1 second for the server


        # Establish network connection
        try:
            # Try to connect to the video and control servers
            self._socket_video.connect((scrcpy.__ADB_LOOPBACK_ADDR__, scrcpy.__ADB_LOOPBACK_PORT__))
            self._socket_control.connect((scrcpy.__ADB_LOOPBACK_ADDR__, scrcpy.__ADB_LOOPBACK_PORT__))
        
        except Exception as e:
            # In exception, close all the connections
            self._socket_video.close()
            self._socket_control.close()
            raise


        # Start video decoding thread
        self._video_background_process = BackgroundVideoCodec(self._socket_video)



    ###### Object handling methods ######

    # Called on command like "with scrcpy(...) as drone:"
    def __enter__(self):
        return self


    # Called when the scope of 'with' has ended
    def __exit__(self, exc_type, exc_value, traceback):
        self.destroy()


    def destroy(self, timeout : float | None = None) -> None:
        """
        Clean all the connections of this class. This include sockets to the
        SCRCPY server, and background threads to process the video stream.

        Args:
            timeout (float | None): maximum time to wait for the server to
                close. if None, waits till server is terminated completely.
        """
        if self._adb_proc is not None:
            self._adb_proc.kill()
            self._adb_proc.wait()
            self._adb_proc = None

        if self._video_background_process is not None:
            if self._video_background_process.isAlive():
                self._video_background_process.stop()
            self._video_background_process = None

        self._socket_video.close()
        self._socket_control.close()
    


    ###### Video methods ######

    def getLastFrame(self) -> numpy.ndarray | None:
        """
        Get the last frame processed from the device.

        Return:
            The last frame if exists, or None if the server is down, or no
            frames where registered.
        """
        return self._video_background_process.read()
    

    def registerVideoListener(self, listener : Callable[[numpy.ndarray, ], None]):
        """
        Register frame listener. A method that will be called on every new
        frame. Be careful for the amount of precess this method do, as the
        it run on the same thread as the codec. Also error in this method will
        result in termination of the codec.

        Args:
            listener ((numpy.ndarray) -> None): User custom method that receives
                the last frame.
        """
        self._video_background_process.registerListener(listener)


    def removeVideoListener(self):
        """
        Remove frame listener, if was set.
        """
        self._video_background_process.unregisterListener()



    ###### Control methods ######


    def __integerToBytes(self, len : int, number : int) -> None:
        """
        Convert integer to bytes array, in big endian (network endian).

        Args:
            len (int): number of bytes to convert to.
            number (int): the number to convert to bytes.

        Internal function.
        """
        return (number).to_bytes(length = len, byteorder = "big", signed = False)


    def __sendTouchEvent(self, event : int, pointerID : int, x : int, y : int):
        """
        Creates and sends touch event to the phone via the SCRCPY server.
        
        Args:
            event (int): the event type to register (MotionEvent.ACTION_*).
            pointerID (int): the ID of the specific touch event.
            x (int): x coordinate of the event.
            y (int): y coordinate of the event.

        Internal function.
        """
        # Get screen dimensions
        frame = self.getLastFrame()
        if frame is None:
            return
        
        screen_height = frame.shape[0]
        screen_width = frame.shape[1]

        # Create touch injection control packet
        message = bytes([0x02])                             # 1 : Control Message Type
        message += bytes([event & 0xFF])                    # 1 : action
        message += self.__integerToBytes(8, pointerID)      # 8 : pointerID
        message += self.__integerToBytes(4, x)              # 4 : position.x
        message += self.__integerToBytes(4, y)              # 4 : position.y
        message += self.__integerToBytes(2, screen_width)   # 2 : position.screenWidth
        message += self.__integerToBytes(2, screen_height)  # 2 : position.screenHeight
        message += self.__integerToBytes(2, 0xFFFF)         # 2 : pressure - 0xFFFF = 1.0f
        message += self.__integerToBytes(4, 0)              # 4 : actionButton
        message += self.__integerToBytes(4, 0)              # 4 : buttons

        self._socket_control.sendall(message)
        


    def touchDown(self, pointerID : int, x : int, y : int):
        """
        Register a touch down event (starting touch event) on the device.

        Args:
            pointerID (int): unique ID that identifies the this touch event.
                Must be a positive 63 bit number.
            x (int): x coordinate of the touch event.
            y (iny): y coordinate of the touch event.
        """
        self.__sendTouchEvent(scrcpy.__ACTION_DOWN__, pointerID, x, y)


    def touchMove(self, pointerID : int, x : int, y : int):
        """
        Register a touch move event (touch is moving) on the device.

        Args:
            pointerID (int): unique ID that identifies the this touch event.
                Must be a positive 63 bit number.
            x (int): x coordinate of the touch event.
            y (iny): y coordinate of the touch event.
        """
        self.__sendTouchEvent(scrcpy.__ACTION_MOVE__, pointerID, x, y)


    def touchUp(self, pointerID : int, x : int, y : int):
        """
        Register a touch up event (ending touch event) on the device.

        Args:
            pointerID (int): unique ID that identifies the this touch event.
                Must be a positive 63 bit number.
            x (int): x coordinate of the touch event.
            y (iny): y coordinate of the touch event.
        """
        self.__sendTouchEvent(scrcpy.__ACTION_UP__, pointerID, x, y)




class BackgroundVideoCodec:
    """
    Capturing the frames in the background,
    so the frame processing doesn't lag the program,
    and the most recent frame will return instantly.

    Currently retrieve the frames only in H264 format.
    SCRCPY supports more formats, but the program doesn't use it for now.
    If there are errors, feel free to change the codec.

    Used internally.
    """

    def __init__(self, sock : socket.socket):
        """
        Initiate background video codec, and start it right away.
        Expecting a open and connected socket to retrieve the frames from.

        Args:
            sock (socket.socket): socket receiving the video from.
        """
        # Internal variables
        self._sock = sock
        self._frame = None
        self._codec = av.codec.context.CodecContext.create('h264', 'r')
        self._live = True
        self._listener = None

        # Starting the background thread
        self._thread = Thread(target = self.__ReadFrames__)
        self._thread.daemon = True
        self._thread.start()


    def __ReadFrames__(self):
        """
        Reads frame in the background
        """
        
        # Iterate while flag is on.
        while self._live:

            # Read data, and close thread if socket is down.
            try:
                data = self._sock.recv(1 << 20) # 1MB
                if len(data) == 0:
                    break
            except ConnectionAbortedError:
                break

            # Iterate thru the packets from the data,
            # and decode the frames from the packets.
            for packet in self._codec.parse(data):
                for frame in self._codec.decode(packet):

                    self._frame = frame.to_ndarray(format = 'bgr24')

                    # Call the listener with the new frame
                    #  save listener in new variable to avoid
                    #  multi-threading bugs.
                    listener : Callable[[numpy.ndarray, ], None] = self._listener

                    if listener:
                        listener(self._frame)

        # Set frame to None, if connection / thread interrupted.    
        self._frame = None

    
    def isAlive(self):
        """ Check if the thread is still alive """
        return self._live or self._thread.is_alive()


    def read(self):
        """ Get the last available frame from this video stream. """
        return self._frame


    def stop(self, timeout : float | None = None):
        """
        Stop the thread. (Also closes the socket)

        Args:
            timeout (float | None): timeout for the operation in seconds,
                or None to wait indefinitely.
        """
        self._live = False
        self._sock.close()
        self._thread.join(timeout)


    def registerListener(self, listener : Callable[[numpy.ndarray, ], None]):
        """ Sets frame listener """
        self._listener = listener


    def unregisterListener(self):
        """ Remove frame listener """
        self._listener = None