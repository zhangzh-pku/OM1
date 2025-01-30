import asyncio
import logging
import os

import dotenv
import typer

from runtime.config import load_config
from runtime.cortex import CortexRuntime
from unitree.unitree_sdk2py.core.channel import ChannelFactoryInitialize

app = typer.Typer()


@app.command()
def start(config_name: str, debug: bool = False) -> None:
    logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)

    # Robot setup, if the env var is actually set to a specific ethernet adapter
    UNIEN0 = os.getenv("UNITREE_WIRED_ETHERNET")
    logging.info(f"Using {UNIEN0} as the Unitree network Ethernet adapter")
    if UNIEN0 is not None and UNIEN0 != "SIM":
        ChannelFactoryInitialize(0, UNIEN0)
        logging.info("Booting Unitree and CycloneDDS")

    config = load_config(config_name)
    runtime = CortexRuntime(config)
    asyncio.run(runtime.run())


if __name__ == "__main__":
    dotenv.load_dotenv()
    app()
