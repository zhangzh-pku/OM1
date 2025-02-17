import base64
import glob
import logging
import os
import threading
import time
from typing import Callable, Optional

import cv2
from om1_utils import ws
from om1_vlm import VideoStream

from .singleton import singleton

root_package_name = __name__.split(".")[0] if "." in __name__ else __name__
logger = logging.getLogger(root_package_name)


class UnitreeRealSenseDevVideoStream(VideoStream):
    """
    Manages video capture and streaming from a camera device.

    Provides functionality to capture video frames from a camera device,
    process them, and stream them through a callback function. Supports
    both macOS and Linux camera devices.

    Parameters
    ----------
    frame_callback : Optional[Callable[[str], None]], optional
        Callback function to handle processed frame data.
        Function receives base64 encoded frame data.
        By default None
    fps : Optional[int], optional
        Frames per second to capture.
        By default 30
    """

    def on_video(self):
        """
        Main video capture and processing loop.

        Captures frames from the camera, encodes them to base64,
        and sends them through the callback if registered.

        Raises
        ------
        Exception
            If video streaming encounters an error
        """

        camindex = self._find_rgb_device()
        logger.info(f"Using camera: {camindex}")

        self._cap = cv2.VideoCapture(camindex)
        if not self._cap.isOpened():
            logger.error(f"Error opening video stream from {camindex}")
            return

        try:
            while self.running:
                ret, frame = self._cap.read()
                if not ret:
                    logger.error("Error reading frame from video stream")
                    time.sleep(0.1)
                    continue

                # Convert frame to base64
                _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                frame_data = base64.b64encode(buffer).decode("utf-8")

                if self.frame_callback:
                    self.frame_callback(frame_data)

                time.sleep(
                    self.frame_delay
                )  # Use calculated frame delay instead of hardcoded value

        except Exception as e:
            logger.error(f"Error streaming video: {e}")
        finally:
            if self._cap:
                self._cap.release()
                logger.info("Released video capture device")

    def _find_rgb_device(self):
        """
        Helper function to find the RGB camera device by scanning for the first device supporting common RGB formats.

        Raises
        ------
        Exception
            If errors are encountered while scanning for the device.
        """
        try:
            video_devices = sorted(
                glob.glob("/dev/video*")
            )  # List all /dev/videoX devices
        except Exception as e:
            logger.error("Failed to list video devices: %s", e)
            return None

        for device in video_devices:
            try:
                cmd = f"v4l2-ctl --device={device} --list-formats"
                formats = os.popen(cmd).read()
            except Exception as e:
                logger.error("Failed to run command '%s': %s", cmd, e)
                continue

            try:
                if "MJPG" in formats or "YUYV" in formats:
                    logger.info(
                        "Found RGB device at %s with formats: %s", device, formats
                    )
                    return device  # Return the first detected RGB stream
            except Exception as e:
                logger.error("Error processing formats for device %s: %s", device, e)

        logger.warning("No RGB device found")
        return None


@singleton
class UnitreeRealSenseDevVLMProvider:
    """
    VLM Provider that handles audio streaming and websocket communication.

     This class implements a singleton pattern to manage audio input streaming and websocket
     communication for vlm services. It runs in a separate thread to handle
     continuous vlm processing.

     Parameters
     ----------
     ws_url : str
         The websocket URL for the ASR service connection.
     fps : int
         Frames per second for the video stream.
    """

    def __init__(self, ws_url: str, fps: int = 30):
        """
        Initialize the VLM Provider.

        Parameters
        ----------
        ws_url : str
            The websocket URL for the VLM service connection.
        """
        self.running: bool = False
        self.ws_client: ws.Client = ws.Client(url=ws_url)
        self.video_stream: VideoStream = UnitreeRealSenseDevVideoStream(
            self.ws_client.send_message, fps=fps
        )
        self._thread: Optional[threading.Thread] = None

    def register_message_callback(self, message_callback: Optional[Callable]):
        """
        Register a callback for processing VLM results.

        Parameters
        ----------
        callback : callable
            The callback function to process VLM results.
        """
        self.ws_client.register_message_callback(message_callback)

    def start(self):
        """
        Start the VLM provider.

        Initializes and starts the websocket client, video stream, and processing thread
        if not already running.
        """
        if self._thread and self._thread.is_alive():
            return

        self.running = True
        self.ws_client.start()
        self.video_stream.start()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        """
        Main loop for the VLM provider.

        Continuously processes video frames and sends them to the VLM service
        for analysis.
        """
        while self.running:
            try:
                time.sleep(0.1)
            except Exception as e:
                logging.error(f"Error in VLM provider: {e}")

    def stop(self):
        """
        Stop the VLM provider.

        Stops the websocket client, video stream, and processing thread.
        """
        self.running = False
        if self._thread:
            self.video_stream.stop()
            self.ws_client.stop()
            self._thread.join(timeout=5)
