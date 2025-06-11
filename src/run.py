import asyncio
import logging
import os
import time

import dotenv
import typer

from runtime.config import load_config
from runtime.cortex import CortexRuntime

app = typer.Typer()


def setup_logging(config_name: str, debug: bool, log_to_file: bool) -> None:
    level = logging.DEBUG if debug else logging.INFO

    logging.getLogger().handlers.clear()

    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)

    handlers = [console_handler]

    if log_to_file:
        os.makedirs("logs", exist_ok=True)

        file_handler = logging.FileHandler(
            f"logs/{config_name}_{time.strftime('%Y-%m-%d_%H-%M-%S')}.log", mode="a"
        )
        file_handler.setLevel(level)
        handlers.append(file_handler)

    logging.basicConfig(level=level, handlers=handlers)


@app.command()
def start(config_name: str, debug: bool = False, log_to_file: bool = False) -> None:
    setup_logging(config_name, debug, log_to_file)

    # Load configuration
    config = load_config(config_name)
    runtime = CortexRuntime(config)

    # Start the runtime
    asyncio.run(runtime.run())


if __name__ == "__main__":
    dotenv.load_dotenv()
    app()
