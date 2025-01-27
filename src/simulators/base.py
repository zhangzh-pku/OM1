import time
from dataclasses import dataclass


@dataclass
class Simulator:
    name: str

    def tick(self) -> None:
        """
        Run the simulator for one tick
        """
        # fallback setting, specific time.sleep value should not matter.
        # the actual vlaue is set in the relvant simulator.py file
        time.sleep(60)
        pass