import asyncio
import random
import logging
import time
import os
from dataclasses import dataclass
from cdp import Cdp, Wallet

from typing import List, Optional

from inputs.base.loop import LoopInput

from providers.io_provider import IOProvider

@dataclass
class Message:
    timestamp: float
    message: str


class WalletCoinbase(LoopInput[float]):
    """
    Queries current ETH balance and reports a balance increase
    """

    def __init__(self):
        # Track IO
        self.io_provider = IOProvider()
        self.messages: list[str] = []

        self.POLL_INTERVAL = 0.5 # seconds between blockchain data updates
        self.COINBASE_WALLET_ID = os.environ.get("COINBASE_WALLET_ID")
        logging.debug(f"Using {self.COINBASE_WALLET_ID} as the coinbase wallet id")

        # Initialize Wallet
        API_KEY = os.environ.get("COINBASE_API_KEY")
        API_SECRET = os.environ.get("COINBASE_API_SECRET")
        Cdp.configure(API_KEY, API_SECRET)

        try:
            # fetch wallet data
            self.wallet = Wallet.fetch(self.COINBASE_WALLET_ID)
            logging.debug(f"Wallet: {self.wallet}")
        except Exception as e:
            logging.error(f"Error fetching Coinbase Wallet data: {e}")
        
        self.ETH_balance = float(self.wallet.balance('eth'))
        self.ETH_balance_previous = self.ETH_balance

        logging.info("Testing: WalletCoinbase: Initialized")


    async def _poll(self) -> List[float]:

        await asyncio.sleep(self.POLL_INTERVAL)

        # randomly simulate ETH inbound transfers for debugging purposes
        if random.randint(0, 10) > 7:
            self.wallet.faucet(asset_id='eth', amount=0.1)

        self.ETH_balance = float(self.wallet.balance('eth'))
        balance_change = self.ETH_balance - self.ETH_balance_previous
        self.ETH_balance_previous = self.ETH_balance

        return [self.ETH_balance, balance_change]

    async def _raw_to_text(self, raw_input: List[float]) -> str:

        balance = raw_input[0]
        balance_change = raw_input[1]

        if balance_change > 0:
            message = f"{time.time():.3f}::You just received {balance_change:.3f} ETH."
        else:
            message = f"{time.time():.3f}::You have {balance:.3f} ETH."

        logging.info(f"WalletEthereum: {message}")
        return Message(timestamp=time.time(), message=message)

    async def raw_to_text(self, raw_input):
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
