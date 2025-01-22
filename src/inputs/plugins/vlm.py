import asyncio
import random
from PIL import Image
from dataclasses import dataclass
import time

from inputs.base.loop import LoopInput

from providers.io_provider import IOProvider

from typing import Optional

@dataclass
class Message:
    timestamp: float
    message: str


class VlmInput(LoopInput[Image.Image]):
    """
    Input from a VLM
    """
    def __init__(self):
        # Track IO
        self.io_provider = IOProvider()

        # Messages buffer
        self.messages: list[Message] = []

    async def _poll(self) -> Image.Image:
        await asyncio.sleep(0.5)
        # this could be a camera or other sensor
        img = Image.new(
            "RGB",
            (100, 100),
            (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)),
        )
        return img

    async def _raw_to_text(self, raw_input: Image.Image) -> Message:
        # now you can use the `raw_input` variable for something, it is of Type Image
        # but for simplementationicity let's not bother with the random image,
        # but just create a string that changes
        num = random.randint(0, 100)
        message = f"I see {num} people. Also, I see a rocket."

        return Message(timestamp=time.time(), message=message)

    async def raw_to_text(self, raw_input: Image.Image):
        """
        Convert raw input to processed text and manage buffer.

        Parameters
        ----------
        raw_input : Optional[str]
            Raw input to be processed
        """
        pending_message = await self._raw_to_text(raw_input)

        if pending_message is not None:
            self.messages.append(pending_message)

    def formatted_latest_buffer(self) -> Optional[str]:
        """
        Format and clear the latest buffer contents.

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
        {latest_message.timestamp:.3f}
        // END
        """

        self.io_provider.add_input(self.__class__.__name__, latest_message.message, latest_message.timestamp)
        self.messages = []

        return result
