import json
import asyncio
from queue import Queue, Empty
import logging
from typing import Dict, Optional, List

from providers.OpenMindProvider import OpenMindProvider
from providers.sleep_ticker_provider import SleepTickerProvider
from inputs.base.loop import LoopInput

class OpenMindInput(LoopInput[str]):
    """OpenMind context query input handler"""
    
    def __init__(self):
        super().__init__()
        self.buffer: List[str] = []
        self.message_buffer: Queue[str] = Queue()

        # Initialize OpenMind provider
        self.provider = OpenMindProvider()
        self.provider.start()
        self.provider.register_message_callback(self._handle_message)

        # Initialize sleep ticker provider
        self.global_sleep_ticker_provider = SleepTickerProvider()

    def _handle_message(self, raw_message: str):
        """
        Process incoming OpenMind API responses
        
        Parameters
        ----------
        raw_message : str
            Raw JSON message received from API
        """
        try:
            json_message: Dict = json.loads(raw_message)
            if "results" in json_message:
                documents = json_message["results"]
                context = '\n\n'.join([r.get('content', {}).get('text', '') for r in documents if r.get('content', {}).get('text', '')])
                self.message_buffer.put(context)
                logging.info(f"Received context: {context}")
        except json.JSONDecodeError:
            logging.error(f"Failed to parse message: {raw_message}")

    async def _poll(self) -> Optional[str]:
        """Poll for new messages"""
        await asyncio.sleep(0.5)
        try:
            message = self.message_buffer.get_nowait()
            return message
        except Empty:
            return None

    async def raw_to_text(self, raw_input):
        """Convert raw input to processed text and manage buffer"""
        text = await self._raw_to_text(raw_input)
        if text is None:
            if len(self.buffer) == 0:
                return None
            else:
                self.global_sleep_ticker_provider.skip_sleep = True

        if text is not None:
            if len(self.buffer) == 0:
                self.buffer.append(text)
            else:
                self.buffer[-1] = f"{self.buffer[-1]} {text}"

    async def _raw_to_text(self, raw_input: str) -> Optional[str]:
        """Convert raw input to text format"""
        return raw_input

    def formatted_latest_buffer(self) -> Optional[str]:
        """Format and clear the latest buffer contents"""
        if len(self.buffer) == 0:
            return None

        result = f"""
{self.__class__.__name__} INPUT
// START
{self.buffer[-1]}
// END
"""
        self.buffer = []
        return result 