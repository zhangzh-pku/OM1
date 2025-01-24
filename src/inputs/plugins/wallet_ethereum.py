import asyncio
import logging
import os
import random
import time
from dataclasses import dataclass
from typing import List, Optional

from web3 import Web3

from inputs.base.loop import LoopInput
from providers.io_provider import IOProvider


@dataclass
class Message:
    timestamp: float
    message: str


class WalletEthereum(LoopInput[float]):
    """
    Queries current ETH balance and reports a balance increase
    """

    def __init__(self):
        # Track IO
        self.io_provider = IOProvider()

        self.ETH_balance = 0
        self.ETH_balance_previous = 0
        self.messages: list[str] = []
        self.eth_info = ""

        self.PROVIDER_URL = "https://eth.llamarpc.com"
        self.POLL_INTERVAL = 0.5  # seconds between blockchain data updates
        self.ACCOUNT_ADDRESS = os.environ.get(
            "ETH_ADDRESS", "0xd8dA6BF26964aF9D7eEd9e03E53415D37aA96045"
        )
        logging.debug(f"Using {self.ACCOUNT_ADDRESS} as the wallet address")

        # Initialize Web3
        self.web3 = Web3(Web3.HTTPProvider(self.PROVIDER_URL))
        if not self.web3.is_connected():
            raise Exception("Failed to connect to Ethereum")

    async def _poll(self) -> List[float]:

        await asyncio.sleep(self.POLL_INTERVAL)

        try:
            # Get latest block data
            block_number = self.web3.eth.block_number

            # Get account data
            balance_wei = self.web3.eth.get_balance(self.ACCOUNT_ADDRESS)
            balance_eth = self.web3.from_wei(balance_wei, "ether")

            self.eth_info = {
                "block_number": int(block_number),
                "address": str(
                    self.ACCOUNT_ADDRESS
                ),  # just to be clear that that's a string
                "balance": float(balance_eth),
            }
            logging.debug(
                f"Block: {self.eth_info['block_number']}, Account Balance: {self.eth_info['balance']:.3f} ETH"
            )

        except Exception as e:
            logging.error(f"Error fetching blockchain data: {e}")

        # randomly simulate ETH inbound transfers for debugging purposes
        random_add_for_debugging = 0
        if random.randint(0, 10) > 7:
            random_add_for_debugging = 1.0

        self.ETH_balance = float(balance_eth) + random_add_for_debugging
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

        self.io_provider.add_input(
            self.__class__.__name__, latest_message.message, latest_message.timestamp
        )
        self.messages = []
        return result
