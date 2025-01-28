import asyncio
import logging
import os
import time
from dataclasses import dataclass
from typing import Optional

from inputs.base.loop import FuserInput
from providers.io_provider import IOProvider
from unitree.unitree_sdk2py.core.channel import (
    ChannelFactoryInitialize,
    ChannelSubscriber,
)
from unitree.unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_


@dataclass
class Message:
    timestamp: float
    message: str


class UnitreeGo2Lowstate(FuserInput[str]):
    """
    Unitree Go2 Air Lowstate bridge.

    Takes specific Unitree CycloneDDS Lowstate messages, converts them to
    text strings, and sends them to the fuser.

    Processes Unitree Lowstate information. These are things like joint position and battery charge.

    Maintains a buffer of processed messages.
    """

    def __init__(self):
        """
        Initialize Unitree bridge with empty message buffer.
        """
        # Track IO
        self.io_provider = IOProvider()

        # Messages buffer
        self.messages: list[Message] = []

        self.UNITREE_WIRED_ETHERNET = os.environ.get("UNITREE_WIRED_ETHERNET", "eno0")
        logging.info(
            f"Using {self.UNITREE_WIRED_ETHERNET} as the network Ethernet adapter"
        )

        # create subscriber
        self.low_state = None
        self.lowstate_subscriber = None

        # Fire up the Unitree system unless adapater is set to "SIM""
        if "SIM" not in self.UNITREE_WIRED_ETHERNET 
            ChannelFactoryInitialize(0, self.UNITREE_WIRED_ETHERNET)
            self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
            self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        self.latest_v = 0.0
        self.latest_a = 0.0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg
        self.latest_v = float(msg.power_v)
        self.latest_a = float(msg.power_a)
        logging.info(f"Battery state voltage: {self.latest_v} current: {self.latest_a}")
        # other things you can read
        # print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
        # print("IMU state: ", msg.imu_state)
        # print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)

    async def _poll(self) -> [float]:
        """
        Poll for new lowstate data.

        Returns
        -------
        [float]
            list of floats
        """

        # Does the complexity of this seem confusing and kinda pointless to you?
        # It's on our radar and your patience is appreciated
        await asyncio.sleep(0.5)

        return [self.latest_v, self.latest_a]

    async def _raw_to_text(self, raw_input: [float]) -> Optional[Message]:
        """
        Process raw lowstate to generate text description.

        Parameters
        ----------
        raw_input : [float]
            Raw lowstate data to be processed

        Returns
        -------
        Message
            Timestamped message containing description
        """
        battery_voltage = raw_input[0]
        if battery_voltage < 26.0:
            message = "WARNING: You are low on energy. SIT DOWN NOW."
            return Message(timestamp=time.time(), message=message)
        elif battery_voltage < 27.2:
            message = "WARNING: You are low on energy. Consider sitting down."
            return Message(timestamp=time.time(), message=message)

    async def raw_to_text(self, raw_input: [float]):
        """
        Convert raw lowstate to text and update message buffer.

        Parameters
        ----------
        raw_input : [float]
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
{self.__class__.__name__} INPUT
// START
{latest_message.message}
// END
"""

        self.io_provider.add_input(
            self.__class__.__name__, latest_message.message, latest_message.timestamp
        )
        self.messages = []

        return result
