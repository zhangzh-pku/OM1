import logging
import threading
import time
from dataclasses import dataclass
from typing import Optional

try:
    from unitree.unitree_sdk2py.go2.sport.sport_client import SportClient
except ImportError:
    logging.error(
        "Unitree SDK or CycloneDDS not found. Please install the unitree_sdk2py package or CycloneDDS."
    )

from .singleton import singleton


@dataclass
class UnitreeGo2State:
    """
    Unitree Go2 State Data Class.

    This class holds the state data for the Unitree Go2 robot.
    """

    state: str
    body_height: float
    foot_raise_height: float
    speed_level: int
    gait: int
    dance: bool
    economic_gait: bool
    continuous_gait: bool


@singleton
class UnitreeGo2StateProvider:
    """
    Unitree Go2 State Provider.

    This class implements a singleton pattern to manage:
        * Unitree Go2 state data using either Zenoh or CycloneDDS
    """

    def __init__(self):
        """
        Robot and sensor configuration
        """
        logging.info("Booting Unitree Go2 State Provider")

        self.sport_client = None
        try:
            self.sport_client = SportClient()
            self.sport_client.Init()
            self.sport_client.SetTimeout(10.0)
            logging.info("Unitree Go2 State Provider initialized successfully")
        except Exception as e:
            logging.error(f"Error initializing Unitree Go2 State Provider: {e}")

        self.go2_state = None
        self._thread = None

        self.start()

    def start(self):
        """
        Start the Unitree Go2 State Provider.

        This method initializes the Unitree Go2 state provider and starts the
        state retrieval process.
        """
        if self._thread and self._thread.is_alive():
            logging.warning("Unitree Go2 State Provider is already running.")
            return

        if not self.sport_client:
            logging.error("SportClient is not initialized.")
            return

        self._thread = threading.Thread(target=self.get_state_periodically, daemon=True)
        self._thread.start()

    def get_state_periodically(self, interval: float = 1.0):
        """
        Periodically retrieve the state of the Unitree Go2 robot.

        Parameters
        ----------
        interval : float
            The time interval in seconds between state retrievals.
        """
        while self._thread and self._thread.is_alive():
            try:
                state = self.get_state()
                if state:
                    self.go2_state = state
                    logging.info(f"Retrieved Unitree Go2 state: {state}")
                else:
                    logging.warning("Failed to retrieve Unitree Go2 state.")
            except Exception as e:
                logging.error(f"Error retrieving Unitree Go2 state: {e}")

            time.sleep(interval)

    def get_state(self) -> Optional[UnitreeGo2State]:
        """
        Get the current state of the Unitree Go2 robot.

        Returns
        -------
        UnitreeGo2State
            The current state of the robot.
        """
        if not self.sport_client:
            logging.error("SportClient is not initialized.")
            return None

        try:
            state = self.sport_client.GetState(
                [
                    "state",
                    "bodyHeight",
                    "footRaiseHeight",
                    "speedLevel",
                    "gait",
                    "dance",
                    "economicGait",
                    "continuousGait",
                ]
            )
            if state[0] != 0:
                logging.error(f"Failed to get state from Unitree Go2: {state[0]}")
                return None

            try:
                return UnitreeGo2State(
                    state=state[1]["state"]["data"],
                    body_height=state[1]["bodyHeight"]["data"],
                    foot_raise_height=state[1]["footRaiseHeight"]["data"],
                    speed_level=state[1]["speedLevel"]["data"],
                    gait=state[1]["gait"]["data"],
                    dance=state[1]["dance"]["data"],
                    economic_gait=state[1]["economicGait"]["data"],
                    continuous_gait=state[1]["continuousGait"]["data"],
                )
            except Exception as e:
                logging.error(f"Error parsing Unitree Go2 state: {e}")
                return None

        except Exception as e:
            logging.error(f"Error getting Unitree Go2 state: {e}")
            return None

    @property
    def state(self) -> Optional[str]:
        """
        Get the current state of the Unitree Go2 robot.

        Returns
        -------
        str or None
            The current state of the robot, or None if not available.
        """
        if self.go2_state is None:
            return None

        return self.go2_state.state
