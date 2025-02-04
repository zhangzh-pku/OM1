import logging
import os
from typing import Any, Dict


def load_unitree(config: Dict[str, Any]):
    """
    Initialize the Unitree robot's network communication channel.

    This function sets up the Ethernet connection for a Unitree robot based on
    the provided configuration or environment variables. It can operate in either
    real hardware or simulation mode.

    Parameters
    ----------
    config : Dict[str, Any]
        Configuration object containing the Unitree Ethernet adapter

    Returns
    -------
    None

    Raises
    ------
    Exception
        If initialization of the Unitree Ethernet channel fails when not in
        simulation mode.

    """
    unitree_ethernet = os.getenv("UNITREE_WIRED_ETHERNET") or config.get(
        "unitree_ethernet", None
    )
    if unitree_ethernet is not None:
        logging.info(
            f"Using {unitree_ethernet} as the Unitree network Ethernet adapter"
        )

        from unitree.unitree_sdk2py.core.channel import ChannelFactoryInitialize

        if unitree_ethernet != "SIM":
            try:
                ChannelFactoryInitialize(0, unitree_ethernet)
            except Exception as e:
                logging.error(f"Failed to initialize Unitree Ethernet channel: {e}")
                raise e
            logging.info("Booting Unitree and CycloneDDS")
        else:
            logging.info("Booting Unitree in simulation mode")
