import asyncio
import random
from PIL import Image
import logging
import time

from inputs.base.loop import LoopInput

from typing import Dict, Optional

class VlmInput(LoopInput[Image.Image]):
    """
    Input from a VLM
    """
    def __init__(self):
        self.messages: list[str] = []

    async def _poll(self) -> Image.Image:
        await asyncio.sleep(0.5)
        # this could be a camera or other sensor
        img = Image.new(
            "RGB",
            (100, 100),
            (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)),
        )
        return img

    async def _raw_to_text(self, raw_input: Image.Image) -> str:
        # now you can use the `raw_input` variable for something, it is of Type Image
        # but for simplementationicity let's not bother with the random image, 
        # but just create a string that changes
        num = random.randint(0, 100)
        message = f"{time.time():.3f}::I see {num} people. Also, I see a rocket."
        logging.debug(f"VlmInput: {message}")
        return message

    async def raw_to_text(self, raw_input):
        """
        Convert raw input to processed text and manage buffer.

        Parameters
        ----------
        raw_input : Optional[str]
            Raw input to be processed
        """
        text = await self._raw_to_text(raw_input)
        if text is None:
            if len(self.messages) == 0:
                return None
            # else:
            #     # Skip sleep if there's already a message in the buffer
            #     self.global_sleep_ticker_provider.skip_sleep = True

        if text is not None:
            if len(self.messages) == 0:
                self.messages.append(text)
            # else:
            # here is where you can implementationement joining older messages
            #     self.buffer[-1] = f"{self.buffer[-1]} {text}"

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

        message = self.messages[-1]
        part = message.split("::")
        ts = part[0]
        ms = part[-1]

        result = f"""{ts}::
        {self.__class__.__name__} INPUT
        // START
        {ms}
        // END
        """

        self.messages = []
        return result
