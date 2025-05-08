import asyncio
import logging
import time
from dataclasses import dataclass
from typing import List, Optional

import zenoh

from inputs.base import SensorConfig
from inputs.base.loop import FuserInput
from providers.io_provider import IOProvider
from zenoh_idl import sensor_msgs


@dataclass
class Message:
    timestamp: float
    message: str


g_battery = None


def listenerBattery(sample):
    battery = sensor_msgs.BatteryState.deserialize(sample.payload.to_bytes())
    battery_percent = int(battery.percentage * 100)
    # print(f"Battery Percentage {battery_percent}")
    global g_battery
    if battery_percent < 5:
        g_battery = "CRITICAL: your battery is almost empty. Immediately move to your charging station and recharge."
    elif battery_percent < 15:
        g_battery = "IMPORTANT: your battery is running low. Consider finding your charging station and recharging."
    else:
        g_battery = None


class TurtleBot4Batt(FuserInput[str]):
    """
    TurtleBot4 Battery inputs.

    Takes specific TurtleBot4 ROS2/Zenoh messages, converts them to
    text strings, and sends them to the fuser.

    Maintains a buffer of processed messages.
    """

    def __init__(self, config: SensorConfig = SensorConfig()):
        super().__init__(config)

        # Track IO
        self.io_provider = IOProvider()

        # Buffer for storing the final output
        self.messages: list[Message] = []

        logging.info("Opening Zenoh TurtleBot4 session...")
        conf = zenoh.Config()
        self.z = zenoh.open(conf)

        logging.info(f"Config: {self.config}")

        self.URID = getattr(self.config, "URID", "default")
        logging.info(f"Using TurtleBot4 URID: {self.URID}")

        logging.info("Creating Zenoh TurtleBot4 Subscriber")
        self.batts = self.z.declare_subscriber(
            f"{self.URID}/c3/battery_state", listenerBattery
        )

        # Simple description of sensor output to help LLM understand its importance and utility
        self.descriptor_for_LLM = "Energy Level"

    async def _poll(self) -> List[str]:
        """
        Poll for new state data.

        Returns
        -------
        List[str]
        """

        # Does the complexity of this seem confusing and kinda pointless to you?
        # It's on our radar and your patience is appreciated
        await asyncio.sleep(2.0)

        logging.info(f"TB4 batt:{g_battery}")

        return g_battery

    async def _raw_to_text(self, raw_input: str) -> Optional[Message]:
        """
        Process raw lowstate to generate text description.

        Parameters
        ----------
        raw_input : List[str]
            State data to be processed

        Returns
        -------
        Message
            Timestamped message containing description
        """
        battery = raw_input

        if battery:
            message = battery
            return Message(timestamp=time.time(), message=message)

    async def raw_to_text(self, raw_input: List[float]):
        """
        Convert raw lowstate to text and update message buffer.

        Parameters
        ----------
        raw_input : List[float]
            Raw lowstate data to be processed
        """
        pending_message = await self._raw_to_text(raw_input)

        if pending_message is not None:
            self.messages.append(pending_message)

    def formatted_latest_buffer(self) -> Optional[str]:
        """
        Format and clear the latest buffer contents.

        Formats the most recent message with timestamp and class name,
        adds it to the IO provider, then clears the buffer.

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
        self.messages = []

        return result
