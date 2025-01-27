import json
import asyncio
from queue import Queue, Empty
import logging
from typing import Dict, Optional, List

from providers.TwitterProvider import TwitterProvider
from providers.sleep_ticker_provider import SleepTickerProvider

from inputs.base.loop import LoopInput

class TwitterInput(LoopInput[str]):
    """Twitt data puller input handler.
    This class manages the input stream from an Twitter data aggregator service, 
    uffering messages.
    Attributes
    ----------
    message_buffer : Queue[str]
        FIFO queue for storing incoming Twitter messages
    twp : TwitterProvider
        Provider for Twitter websocket connection
    global_sleep_ticker_provider : SleepTickerProvider
        Provider for managing sleep ticks
    buffer : List[str]
        Internal buffer for storing processed twitter data
    """
    def __init__(self):
        super().__init__()

        # Buffer for storing the final output
        self.buffer: List[str] = []

        # Buffer for storing messages
        self.message_buffer: Queue[str] = Queue()

        # Initialize ASR provider
        self.twp: TwitterProvider = TwitterProvider(ws_url="wss://api.openmind.org/api/core/query") 
        self.twp.start()
        self.twp.register_message_callback(self._handle_twp_message)

        # Initialize sleep ticker provider
        self.global_sleep_ticker_provider = SleepTickerProvider()

    def _handle_twitter_message(self, raw_message: str):
        """Handle incoming WebSocket messages"""
        try:
            message = json.loads(raw_message)
            if "results" in message:
                documents = message["results"]
                formatted_context = '\n\n'.join([r.get('content', {}).get('text', '') for r in documents if r.get('content', {}).get('text', '')])
                self.message_buffer.put(formatted_context)
                logging.info(f"Received context: {formatted_context}")
        except json.JSONDecodeError:
            logging.error(f"Invalid JSON message: {raw_message}")
            
    
    async def _poll(self) -> Optional[str]:
        """
        Poll for new messages in the buffer.
        Returns
        -------
        Optional[str]
            Message from the buffer if available, None otherwise
        """
        await asyncio.sleep(0.5)
        try:
            message = self.message_buffer.get_nowait()
            return message
        except Empty:
            return None

    async def _raw_to_text(self, raw_input: str) -> str:
        """
        Convert raw input to text format.
        Parameters
        ----------
        raw_input : str
            Raw input string to be converted
        Returns
        -------
        Optional[str]
            Converted text or None if conversion fails
        """
        return raw_input

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
            if len(self.buffer) == 0:
                return None
            else:
                # Skip sleep if there's already a message in the buffer
                self.global_sleep_ticker_provider.skip_sleep = True

        if text is not None:
            if len(self.buffer) == 0:
                self.buffer.append(text)
            else:
                self.buffer[-1] = f"{self.buffer[-1]} {text}"

    def formatted_latest_buffer(self) -> Optional[str]:
        """
        Format and clear the latest buffer contents.
        Returns
        -------
        Optional[str]
            Formatted string of buffer contents or None if buffer is empty
        """
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