import asyncio
import logging
import threading
import platform
import multiprocessing
import time
import os

import dotenv
import typer

from runtime.config import load_config
from runtime.cortex import CortexRuntime
from runtime.robotics import load_unitree
from simulators.plugins.simulator_loop import SimulatorLoop

app = typer.Typer()

@app.command()
def start(config_name: str, debug: bool = False) -> None:
    logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)

    # Load configuration
    config = load_config(config_name)
    _ = load_unitree(config)
    runtime = CortexRuntime(config)

    # Handle platform-specific initialization
    if platform.system() == 'Darwin':
        # Create and start the asyncio thread
        async def run_async():
            await runtime.run()

        asyncio_thread = threading.Thread(target=lambda: asyncio.run(run_async()))
        asyncio_thread.daemon = True
        asyncio_thread.start()

        # Run simulator in main thread
        simulator_loop = SimulatorLoop(runtime)
        simulator_loop.run()
    else:
        # On other platforms, run normally
        asyncio.run(runtime.run())

if __name__ == "__main__":
    # Enable multiprocessing support
    multiprocessing.freeze_support()
    dotenv.load_dotenv()
    app()
