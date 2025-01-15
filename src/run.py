import asyncio
import typer
import dotenv
import logging
from runtime.config import load_config
from runtime.cortex import CortexRuntime
from runtime.global_state import GlobalState

app = typer.Typer()


@app.command()
def start(config_name: str, debug: bool = False) -> None:
    logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)
    config = load_config(config_name)

    state = GlobalState()

    runtime = CortexRuntime(config)
    asyncio.run(runtime.run())


if __name__ == "__main__":
    dotenv.load_dotenv()
    app()
