import asyncio
import logging

import dotenv
import typer

from actions.tweet.first_boot import send_first_boot_tweet
from runtime.config import load_config
from runtime.cortex import CortexRuntime

app = typer.Typer()


@app.command()
def start(config_name: str, debug: bool = False) -> None:
    logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)
    config = load_config(config_name)
    runtime = CortexRuntime(config)
    asyncio.run(runtime.run())


if __name__ == "__main__":
    dotenv.load_dotenv()
    app()
