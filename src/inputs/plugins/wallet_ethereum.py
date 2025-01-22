import asyncio
import random
import logging
import time

from typing import Dict, Optional

from inputs.base.loop import LoopInput

class WalletEthereum(LoopInput[float]):
    """
    Queries current ETH balance and reports a balance increase
    """

    def __init__(self):
        self.ETH_balance = 0
        self.ETH_balance_previous = 0
        self.messages: list[str] = []

        # connect to Ethereum here
        # ETH api setup

    async def _poll(self) -> [float,float]:
        await asyncio.sleep(1)
        
        # query ETH balance here
        # faking it for now
        self.ETH_balance = random.randint(0, 255)
        balance_change = self.ETH_balance - self.ETH_balance_previous
        self.ETH_balance_previous = self.ETH_balance

        return [self.ETH_balance, balance_change]

    async def _raw_to_text(self, raw_input: [float, float]) -> str:
        
        balance = raw_input[0]
        balance_change = raw_input[1]

        if balance_change > 0:
            message = f"{time.time():.3f}::You just received {balance_change:.3f} ETH."
        else:
            message = f"{time.time():.3f}::You have {balance:.3f} ETH."

        logging.info(f"WalletEthereum: {message}")
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
            # if len(self.messages) == 0:
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
