import asyncio
import random
from PIL import Image


from input.base.loop import LoopInput


class VlmInput(LoopInput[Image.Image]):
    """
    Input from a VLM
    """

    async def _poll(self) -> Image.Image:
        await asyncio.sleep(0.5)
        img = Image.new(
            "RGB",
            (100, 100),
            (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)),
        )
        return img

    async def _raw_to_text(self, raw_input: Image.Image) -> str:
        num = random.randint(0, 100)
        message = f"I see {num} people. One of them is frowning. Also, I see a rocket."
        return message
