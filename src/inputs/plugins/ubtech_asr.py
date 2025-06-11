import json
import logging
import time
from queue import Empty, Queue
from typing import Dict, List, Optional

from inputs.base import SensorConfig
from inputs.base.loop import FuserInput
from providers.io_provider import IOProvider
from providers.sleep_ticker_provider import SleepTickerProvider
from providers.ubtech_asr_provider import UbtechASRProvider 

LANGUAGE_CODE_MAP: dict = {
    "english": "en",
    "chinese": "zh",
    "korean": "ko",
}

class UbtechASRInput(FuserInput[str]):
    """
    Ubtech Robot ASR input handler that uses the UbtechASRProvider.
    """

    def __init__(self, config: SensorConfig = SensorConfig()):
        super().__init__(config)
        self.messages: List[str] = []
        self.descriptor_for_LLM = "Voice"
        self.io_provider = IOProvider()
        self.message_buffer: Queue[str] = Queue()
        self.global_sleep_ticker_provider = SleepTickerProvider()

        # Get config for the provider
        self.robot_ip = getattr(self.config, "robot_ip", None)
        self.language = getattr(self.config, "language", "english").strip().lower()
        self.language_code = LANGUAGE_CODE_MAP.get(self.language, "en")

        # Initialize and start the provider
        self.asr = UbtechASRProvider(robot_ip=self.robot_ip, language_code=self.language_code)
        self.asr.start()
        # Register our internal method as the callback for the provider
        self.asr.register_message_callback(self._handle_asr_message)


    def _handle_asr_message(self, message: Optional[str]):
        """Callback function to handle ASR messages from the provider."""
        if message and len(message.split()) >= 1:  
            logging.info("Detected ASR message: %s", message)
            self.message_buffer.put(message)
        else:
            logging.debug("Ignored empty or malformed ASR message: %s", message)

    async def _poll(self) -> Optional[str]:
        """Poll for new messages, resuming ASR only when the system is ready."""
        try:
            # Attempt to get a message from the buffer that the provider has left.
            return self.message_buffer.get_nowait()
        except Empty:
            # The buffer is empty. This is our signal that the system has processed
            # the previous command and is now ready for a new one. If the ASR is
            # paused, it's time to resume it.
            if self.asr.paused:
                logging.debug("Buffer empty, resuming ASR for next command.")
                self.asr.resume()
            return None

    # The rest of the methods are for formatting and can be copied directly
    async def _raw_to_text(self, raw_input: str) -> str:
        return raw_input

    async def raw_to_text(self, raw_input: str):
        pending_message = await self._raw_to_text(raw_input)
        if pending_message is None:
            if len(self.messages) != 0:
                self.global_sleep_ticker_provider.skip_sleep = True
        else:
            if len(self.messages) == 0:
                self.messages.append(pending_message)
            else:
                self.messages[-1] = f"{self.messages[-1]} {pending_message}"

    def formatted_latest_buffer(self) -> Optional[str]:
        if len(self.messages) == 0:
            return None

        result = f"""
INPUT: {self.descriptor_for_LLM}
// START
{self.messages[-1]}
// END
"""
        self.io_provider.add_input(
            self.descriptor_for_LLM, self.messages[-1], time.time()
        )
        self.messages = []
        return result