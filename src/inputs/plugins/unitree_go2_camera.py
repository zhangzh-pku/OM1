import asyncio
import logging
import random
import time
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from PIL import Image

from inputs.base import SensorOutputConfig
from inputs.base.loop import FuserInput
from providers.io_provider import IOProvider

try:
    from unitree.unitree_sdk2py.go2.video.video_client import VideoClient
except ImportError:
    logging.warning(
        "Unitree SDK not found. Please install the Unitree SDK to use this plugin."
    )
    VideoClient = None


@dataclass
class Message:
    timestamp: float
    message: str


class UnitreeGo2Camera(FuserInput[str]):
    """
    Unitree Go2 Air Camera bridge.
    Takes specific Unitree Go2 Air video messages, converts them to
    text strings, and sends them to the fuser.
    Processes Unitree video information. These are things like video frames.
    Maintains a buffer of processed messages.
    """

    def __init__(self, config: SensorOutputConfig = SensorOutputConfig()):
        """
        Initialize Unitree bridge with empty message buffer.
        """
        super().__init__(config)
        # Track IO
        self.io_provider = IOProvider()
        # Messages buffer
        self.messages: list[Message] = []
        # create subscriber
        self.video_client = VideoClient()
        self.video_client.Init()

    async def _poll(self) -> str:
        """
        Poll for new video input.
        Currently generates random video frames for testing.
        In production, this would interface with camera or sensor.

        Returns
        -------
        str
            Generated or captured video frame
        """
        await asyncio.sleep(0.5)
        code, data = self.video_client.GetImageSample()

        if code == 0:
            # Convert to numpy image
            image_data = np.frombuffer(bytes(data), dtype=np.uint8)
            image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)

            # Display image
            cv2.imshow("front_camera", image)

            # Add a waitKey to properly display the window and check for exit
            key = cv2.waitKey(1)
            if key == 27:  # ESC key to break the loop
                cv2.destroyAllWindows()
                return "Exit requested"

            return "Frame processed successfully"
        return "No frame available"

    async def _raw_to_text(self, raw_input: Image.Image) -> Message:
        """
        Process raw image input to generate text description.

        Parameters
        ----------
        raw_input : Image.Image
            Input image to process

        Returns
        -------
        Message
            Timestamped message containing description
        """
        # You can use the `raw_input` variable for something, it is of Type Image
        # But for simpicity let's just create a string that changes
        num = random.randint(0, 100)
        message = f"DUMMY VLM - FAKE DATA - I see {num} people. Also, I see a rocket."

        return Message(timestamp=time.time(), message=message)

    async def raw_to_text(self, raw_input: Image.Image):
        """
        Convert raw image to text and update message buffer.

        Parameters
        ----------
        raw_input : Image.Image
            Raw image to be processed
        """
        pending_message = await self._raw_to_text(raw_input)

        if pending_message is not None:
            self.messages.append(pending_message)

    def formatted_latest_buffer(self) -> Optional[str]:
        """
        Format and clear the latest buffer contents.

        Formats the most recent message with timestamp and class name,
        adds it to the IO provider, then clears the buffer.

        Returns
        -------
        Optional[str]
            Formatted string of buffer contents or None if buffer is empty
        """
        if len(self.messages) == 0:
            return None

        latest_message = self.messages[-1]

        result = f"""
{self.__class__.__name__} INPUT
// START
{latest_message.message}
// END
"""

        self.io_provider.add_input(
            self.__class__.__name__, latest_message.message, latest_message.timestamp
        )
        self.messages = []

        return result
