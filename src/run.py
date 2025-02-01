import asyncio
import logging
import threading
import platform
import multiprocessing
import time
import queue
import os
import pygame

import dotenv
import typer

from runtime.config import load_config
from runtime.cortex import CortexRuntime
from runtime.robotics import load_unitree

app = typer.Typer()

class MacOSSimulatorLoop:
    def __init__(self, runtime):
        self.runtime = runtime
        self.running = True
        self.event_queue = queue.Queue()
        self.clock = pygame.time.Clock()

    def run(self):
        """Run the simulator loop on the main thread"""
        try:
            # Set environment variables for macOS
            os.environ['SDL_VIDEODRIVER'] = 'cocoa'
            os.environ['SDL_THREADSAFE'] = '1'
            os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
            os.environ['SDL_VIDEO_CENTERED'] = '1'
            
            # Run the simulator loop
            while self.running:
                if hasattr(self.runtime.config, 'simulators'):
                    for simulator in self.runtime.config.simulators:
                        try:
                            simulator.tick()
                        except Exception as e:
                            print(f"Error in simulator tick: {e}")
                            if "display" in str(e).lower():
                                continue
                            self.running = False
                            break
                
                # Maintain consistent frame rate
                self.clock.tick(60)

        except KeyboardInterrupt:
            self.cleanup()
        except Exception as e:
            print(f"Error in simulator loop: {e}")
            self.cleanup()

    def cleanup(self):
        """Clean up resources"""
        self.running = False
        if hasattr(self.runtime.config, 'simulators'):
            for simulator in self.runtime.config.simulators:
                try:
                    simulator.cleanup()
                except Exception as e:
                    print(f"Error cleaning up simulator: {e}")

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

        # Run simulator in main thread with custom loop
        simulator_loop = MacOSSimulatorLoop(runtime)
        simulator_loop.run()
    else:
        # On other platforms, run normally
        asyncio.run(runtime.run())

if __name__ == "__main__":
    # Enable multiprocessing support
    multiprocessing.freeze_support()
    dotenv.load_dotenv()
    app()
