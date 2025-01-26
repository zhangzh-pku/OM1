import time
from dataclasses import dataclass


@dataclass
class Simulator:
    name: str

    def tick(self) -> None:
        """
        Run the simulator for one tick
        """
        time.sleep(60)
        pass
