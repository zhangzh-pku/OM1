import logging
import json
import asyncio
import aiohttp
import threading
from typing import Callable, Optional
from .singleton import singleton

@singleton
class OpenMindProvider:
    """Provider for OpenMind API context queries"""
    
    def __init__(self, api_url: str = "https://api.openmind.org/api/core/query"):
        """
        Initialize the OpenMind Provider.
        
        Parameters
        ----------
        api_url : str
            The API endpoint URL for context queries
        """
        self.running = False
        self.api_url = api_url
        self._message_callback = None
        self._thread = None
        self._loop = None
        self.session = None
        self._query_queue = asyncio.Queue()
        logging.info(f"OpenMind provider initialized with URL: {api_url}")

    def register_message_callback(self, callback: Callable[[str], None]):
        """Register callback for processing query results"""
        self._message_callback = callback

    def start(self):
        """Start the provider in a new thread with its own event loop"""
        if self._thread and self._thread.is_alive():
            return

        self.running = True
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()
        logging.info("OpenMind provider thread started")

    def _run_async_loop(self):
        """Run the event loop in the background thread"""
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        try:
            self._loop.run_until_complete(self._run_provider())
        except Exception as e:
            logging.error(f"Provider error: {e}")
        finally:
            if self.session:
                self._loop.run_until_complete(self.session.close())
            self._loop.close()

    async def _run_provider(self):
        """Main provider loop"""
        # Initialize session
        timeout = aiohttp.ClientTimeout(total=10)
        self.session = aiohttp.ClientSession(timeout=timeout)
        logging.info("HTTP session initialized")
        
        # Process queries while running
        while self.running:
            try:
                # Get query from queue with timeout
                try:
                    query = await asyncio.wait_for(self._query_queue.get(), timeout=1.0)
                    await self._do_query(query)
                except asyncio.TimeoutError:
                    continue
                except Exception as e:
                    logging.error(f"Error processing query: {e}")
            except Exception as e:
                logging.error(f"Provider loop error: {e}")
                await asyncio.sleep(1)

    async def _do_query(self, query: str):
        """Internal method to perform the actual query"""
        if not self.session:
            logging.error("HTTP session not initialized")
            return
            
        try:
            logging.info(f"Sending query: {query}")
            
            async with self.session.post(
                self.api_url,
                json={"query": query},
                headers={"Content-Type": "application/json"}
            ) as response:
                if response.status == 200:
                    data = await response.json()
                    if self._message_callback:
                        self._message_callback(json.dumps(data))
                    logging.info(f"Successfully received context for: {query}")
                else:
                    error_text = await response.text()
                    logging.error(f"Query failed with status {response.status}: {error_text}")
                        
        except asyncio.TimeoutError:
            logging.error("Request timed out")
        except Exception as e:
            logging.error(f"Error querying context: {e}")

    async def query_context(self, query: str):
        """Queue a query for processing"""
        # Create a new queue if needed (in case we're called before start)
        if not hasattr(self, '_query_queue'):
            self._query_queue = asyncio.Queue()
            
        # Put the query in the queue
        await self._query_queue.put(query)
        logging.info(f"Queued query: {query}")

    def stop(self):
        """Stop the provider and cleanup resources"""
        self.running = False
        if self._thread:
            self._thread.join(timeout=5)
            self._thread = None
        if self._loop:
            self._loop.stop()
            self._loop = None
        logging.info("OpenMind provider stopped") 