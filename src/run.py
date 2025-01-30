import asyncio
import logging
import typer

from actions.tweet.first_boot import send_first_boot_tweet
from runtime.config import load_config
from runtime.cortex import CortexRuntime

print("Starting omOS...")

app = typer.Typer()

@app.command()
def start(config_name: str, query: str = None, debug: bool = False):
    """Start omOS with optional query"""
    print(f"Running start command with config: {config_name}")
    
    # Set up logging
    logging_level = logging.DEBUG if debug else logging.INFO
    logging.basicConfig(
        level=logging_level,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    # Load config first
    logging.info(f"Loading config: {config_name}")
    config = load_config(config_name)
    
    # Try to send first boot tweet
    logging.info("Attempting to send first boot tweet...")
    send_first_boot_tweet(config)

    runtime = CortexRuntime(config)
    asyncio.run(runtime.run())
       

if __name__ == "__main__":
    print("Running main")
    app()
