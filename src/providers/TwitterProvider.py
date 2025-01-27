import logging
import json
import asyncio
import websockets
from typing import Callable, Optional
from dotenv import load_dotenv

class TwitterProvider:
    """Provider for Twitter API and WebSocket context queries"""
    
    def __init__(self, ws_url="wss://api.openmind.org/api/core/query"):
        self.ws_url = ws_url
        self.ws = None
        self._message_callback = None
        self.running = False
        self._ws_task = None

    def register_message_callback(self, callback: Callable[[str], None]):
        """Register callback for incoming WebSocket messages"""
        self._message_callback = callback

    def start(self):
        """Start WebSocket connection in background"""
        if self.running:
            return
            
        self.running = True
        self._ws_task = asyncio.create_task(self._ws_handler())
        logging.info("Twitter provider WebSocket started")

    async def stop(self):
        """Stop WebSocket connection"""
        self.running = False
        if self._ws_task:
            self._ws_task.cancel()
        if self.ws:
            await self.ws.close()
        logging.info("Twitter provider WebSocket stopped")

    async def _ws_handler(self):
        """Handle WebSocket connection and messages"""
        while self.running:
            try:
                async with websockets.connect(self.ws_url) as websocket:
                    self.ws = websocket
                    logging.info("WebSocket connected")
                    
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