import asyncio
import logging
import time

import dotenv
import typer

from runtime.config import load_config
from runtime.cortex import CortexRuntime

app = typer.Typer()


@app.command()
def start(config_name: str, debug: bool = False, log_to_file: bool = False) -> None:
    logging.basicConfig(
        level=logging.DEBUG if debug else logging.INFO,
        **(
            dict(filename=f"om1-{time.time()}.log", filemode="a") if log_to_file else {}
        ),
    )

    # Load configuration
    config = load_config(config_name)
    runtime = CortexRuntime(config)

    # Start the runtime
    asyncio.run(runtime.run())


if __name__ == "__main__":
    dotenv.load_dotenv()
    app()
