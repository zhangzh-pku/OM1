import asyncio
from dataclasses import dataclass


@dataclass
class Simulator:
    name: str

    async def tick(self) -> None:
        """
        Run the simulator for one tick
        """
        await asyncio.sleep(60)
