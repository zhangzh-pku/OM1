import asyncio
import logging

import dotenv
import typer

from runtime.config import load_config
from runtime.cortex import CortexRuntime
from actions.tweet.first_boot import send_first_boot_tweet

app = typer.Typer()


@app.command()
def start(config_name: str, query: str = None, debug: bool = False):
    """Start omOS with optional query"""
    logging.basicConfig(level=logging.DEBUG if debug else logging.INFO)
    
    # Load config first
    config = load_config(config_name)
    
    # Try to send first boot tweet for any config with tweet capability
    send_first_boot_tweet(config)
    
    if config_name == "twitter" and query:
        # Run with query
        runtime = CortexRuntime(config)
        asyncio.run(runtime.run())
        return
        
    # Normal startup
    runtime = CortexRuntime(config)
    asyncio.run(runtime.run())
