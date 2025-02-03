import asyncio
import logging

import dotenv
import typer

from runtime.config import load_config
from runtime.cortex import CortexRuntime
from runtime.robotics import load_unitree

app = typer.Typer()


@app.command()
def start(config_name: str, debug: bool = False) -> None:
    logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)

    # Load configuration
    config = load_config(config_name)
    _ = load_unitree(config)
    runtime = CortexRuntime(config)

    # Run normally using orchestrator's thread management
    asyncio.run(runtime.run())


if __name__ == "__main__":
    dotenv.load_dotenv()
    app()
