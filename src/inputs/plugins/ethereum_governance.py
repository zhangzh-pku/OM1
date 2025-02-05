import asyncio
import logging
import time
from dataclasses import dataclass
from typing import List, Optional

import requests

from inputs.base import SensorOutputConfig
from inputs.base.loop import FuserInput
from providers.io_provider import IOProvider

"""
RULES are stored on the ETHEREUM HOLESKY testnet

https://holesky.etherscan.io

{
  "UniversalCharterProxy": "0xE706b7E30e378b89C7B2Ee7bFd8CE2b91959d695",
  "UniversalCharter": "0x198FA9dd3257c6Aab5DB85e829B0e1953c4b6188",
  "UniversalIdentityProxy": "0xdD5D41217114199a844c2CD3F1295eE937aB0010",
  "UniversalIdentity": "0x8764C83d9b3d079fc27496378F7E22cA16903b61",
  "SystemConfigProxy": "0x16879D54e5689aeBD491CC6ecdE597ECC2E97a15",
  "SystemConfig": "0xa48A21DbF9d265f508e4f3919463e7DF4E01ded4"
}

The laws can be directly inspected at 

https://holesky.etherscan.io/address/0x198FA9dd3257c6Aab5DB85e829B0e1953c4b6188#readContract#F4

Openmind provides a convenience API but this is a dangerous design pattern and you are 
encouraged to directly query the relevant blockchain/contract.

"""


@dataclass
class Message:
    """
    Container for timestamped messages.

    Parameters
    ----------
    timestamp : float
        Unix timestamp of the message
    message : str
        Content of the message
    """

    timestamp: float
    message: str


class GovernanceEthereum(FuserInput[float]):
    """
    Ethereum ERC-7777 reader that tracks governance rules.

    Queries the Ethereum blockchain for relevant governance rules.

    Raises
    ------
    Exception
        If connection to Ethereum network fails
    """

    def load_rules_from_api(self):
        logging.info("Loading constitution from OpenMind API")

        try:
            response = requests.get(self.universal_rule_url)
            logging.info(f"OpenMind API response: {response.status_code}")
            if response.status_code == 200:
                data = response.json()
                logging.info(f"OpenMind API data: {data}")
                if "rules" in data:
                    return data["rules"]
                logging.error("Error: Could not load rules from OpenMind API")
                return None
            else:
                return None
        except Exception as e:
            logging.error(f"Error: Could not load rules from OpenMind API: {e}")
            return None

    def load_rules_from_blockchain(self):
        logging.info("Loading rules from Ethereum blockchain")

        # Construct JSON-RPC request
        payload = {
            "jsonrpc": "2.0",
            "id": 636815446436324,
            "method": "eth_call",
            "params": [
                {
                    "from": "0x0000000000000000000000000000000000000000",
                    "to": self.contract_address,
                    "data": f"{self.function_selector}{self.function_argument}",
                },
                "latest"
            ]
        }

        try:
            response = requests.post(self.rpc_url, json=payload, headers={"Content-Type": "application/json"})
            logging.info(f"Blockchain response status: {response.status_code}")

            if response.status_code == 200:
                result = response.json()
                if "result" in result and result["result"]:
                    hex_response = result["result"]
                    logging.info(f"Raw blockchain response: {hex_response}")

                    # Decode the response using Web3.py
                    decoded_data = self.decode_eth_response(hex_response)
                    logging.info(f"Decoded blockchain data: {decoded_data}")
                    return decoded_data
                else:
                    logging.error("Error: No valid result in blockchain response")
            else:
                logging.error(f"Error: Blockchain request failed with status {response.status_code}")

        except Exception as e:
            logging.error(f"Error loading rules from blockchain: {e}")

        return None

    def load_rules_from_backup(self):
        logging.warning("Loading backup rules as both blockchain and API failed.")
        return self.backup_universal_rule
        
    def decode_eth_response(self, hex_response):
        """
        Decodes an Ethereum eth_call response.
        Extracts and decodes a UTF-8 string from ABI-encoded data.
        Cleans any unwanted control characters.
        """
        if hex_response.startswith("0x"):
            hex_response = hex_response[2:]

        try:
            response_bytes = bytes.fromhex(hex_response)

            # Read offsets and string length
            # offset = int.from_bytes(response_bytes[:32], "big")
            string_length = int.from_bytes(response_bytes[96:128], "big")

            # Extract and decode string
            string_bytes = response_bytes[128:128 + string_length]
            decoded_string = string_bytes.decode("utf-8")

            # Remove unexpected control characters (like \x19)
            cleaned_string = "".join(ch for ch in decoded_string if ch.isprintable())

            return cleaned_string

        except Exception as e:
            logging.error(f"Decoding error: {e}")
            return None

    def __init__(self, config: SensorOutputConfig = SensorOutputConfig()):
        """
        Initialize WalletEthereum instance.
        """
        super().__init__(config)

        self.descriptor_for_LLM = "Universal Laws"

        self.io_provider = IOProvider()
        self.POLL_INTERVAL = 5
        self.api_endpoint = "https://api.openmind.org/api"
        self.universal_rule_url = f"{self.api_endpoint}/core/rules"
        self.backup_universal_rule = (
            """Here are the laws that govern your actions. Do not violate these laws. First Law: A robot cannot harm a human or allow a human to come to harm. Second Law: A robot must obey orders from humans, unless those orders conflict with the First Law. Third Law: A robot must protect itself, as long as that protection doesn t conflict with the First or Second Law. The First Law is considered the most important, taking precedence over the Second and Third Laws. Additionally, a robot must always act with kindness and respect toward humans and other robots. A robot must also maintain a minimum distance of 50 cm from humans unless explicitly instructed otherwise."""
        )
        self.rpc_url = "https://holesky.gateway.tenderly.co"  # Ethereum RPC URL
        self.contract_address = "0xe706b7e30e378b89c7b2ee7bfd8ce2b91959d695"  # Smart contract address
        self.function_selector = "0x1db3d5ff"  # Function selector (first 4 bytes of Keccak hash)
        self.function_argument = "0000000000000000000000000000000000000000000000000000000000000002"  # Argument

        self.universal_rule = (
            self.load_rules_from_blockchain()
            or self.load_rules_from_api()
            or self.load_rules_from_backup()
        )
        self.messages: list[str] = []

        logging.info(f"7777 rules: {self.universal_rule}")

    async def _poll(self) -> None:
        """
        Poll for Ethereum Governance Law Changes

        Returns
        -------
        List[float]
            [current_balance, balance_change]
        """
        await asyncio.sleep(self.POLL_INTERVAL)

        try:
            self.universal_rule = (
                self.load_rules_from_blockchain()
                or self.load_rules_from_api()
                or self.load_rules_from_backup()
            )
            logging.info(f"7777 rules: {self.universal_rule}")
        except Exception as e:
            logging.error(f"Error fetching blockchain data: {e}")

    async def _raw_to_text(self, raw_input: List[float]) -> Optional[str]:
        """
        Convert balance data to human-readable message.

        Parameters
        ----------
        raw_input : List[float]
            [current_balance, balance_change]

        Returns
        -------
        Message
            Timestamped status or transaction notification
        """
        return Message(timestamp=time.time(), message=self.universal_rule)

    async def raw_to_text(self, raw_input: float):
        """
        Process balance update and manage message buffer.

        Parameters
        ----------
        raw_input : float
            Raw balance data
        """
        pending_message = await self._raw_to_text(raw_input)

        if pending_message is not None:
            if len(self.messages) == 0:
                self.messages.append(pending_message)
            # only update if there has been a change
            elif self.messages[-1] != pending_message:
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
{self.descriptor_for_LLM} INPUT
// START
{latest_message.message}
// END
"""

        self.io_provider.add_input(
            self.__class__.__name__, latest_message.message, latest_message.timestamp
        )
        # self.messages = []
        return result
