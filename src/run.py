import asyncio
import logging
import multiprocessing
import platform

import dotenv
import typer

from runtime.config import load_config
from runtime.cortex import CortexRuntime
from runtime.robotics import load_unitree
from simulators.plugins.platform_handlers import init_platform

app = typer.Typer()

@app.command()
def start(config_name: str, debug: bool = False) -> None:
    logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)

    # Initialize platform-specific settings
    init_platform()

    # Load configuration
    config = load_config(config_name)
    _ = load_unitree(config)
    runtime = CortexRuntime(config)

    # Run normally using orchestrator's thread management
    asyncio.run(runtime.run())

if __name__ == "__main__":
    multiprocessing.freeze_support()
    dotenv.load_dotenv()
    app()
