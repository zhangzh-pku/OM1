import logging
import json
import asyncio
import websockets
import threading
from typing import Callable, Optional
from dotenv import load_dotenv
from .singleton import singleton

@singleton
class TwitterProvider:
    """Provider for Twitter API and WebSocket context queries"""
    
    def __init__(self, ws_url="wss://api.openmind.org/api/core/query"):
        self.running = False
        self.ws_url = ws_url
        self.ws = None
        self._message_callback = None
        self._thread = None
        self._loop = None

    def register_message_callback(self, callback: Callable[[str], None]):
        """Register callback for incoming WebSocket messages"""
        self._message_callback = callback

    def start(self):
        """Start the provider in a new thread with its own event loop"""
        if self._thread and self._thread.is_alive():
            return

        self.running = True
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()
        logging.info("Twitter provider thread started")

    def _run_async_loop(self):
        """Run the event loop in the background thread"""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        try:
            self._loop.run_until_complete(self._ws_handler())
        except Exception as e:
            logging.error(f"WebSocket handler error: {e}")
        finally:
            self._loop.close()

    async def stop(self):
        """Stop the provider and cleanup"""
        self.running = False
        if self._thread:
            self._thread.join(timeout=5)
            self._thread = None
        if self._loop:
            self._loop.stop()
            self._loop = None
        if self.ws:
            await self.ws.close()
        logging.info("Twitter provider stopped")

    async def _ws_handler(self):
        """Handle WebSocket connection and messages"""
        while self.running:
            try:
                async with websockets.connect(self.ws_url) as websocket:
                    self.ws = websocket
                    logging.info(f"WebSocket connected to {self.ws_url}")
                    
                    while self.running:
                        message = await websocket.recv()
                        if self._message_callback:
                            self._message_callback(message)
                            
            except websockets.exceptions.ConnectionClosed:
                logging.warning("WebSocket connection closed, reconnecting...")
                await asyncio.sleep(1)
            except Exception as e:
                logging.error(f"WebSocket error: {e}")
                await asyncio.sleep(1)

    async def query_context(self, query: str):
        """Send query through WebSocket"""
        if not self.ws:
            logging.error("WebSocket not connected")
            return
            
        try:
            message = json.dumps({"query": query})
            await self.ws.send(message)
            logging.info(f"Sent query: {query}")
        except Exception as e:
            logging.error(f"Failed to send query: {e}")