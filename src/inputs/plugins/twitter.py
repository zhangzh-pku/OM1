import asyncio
from queue import Empty, Queue
import logging
from typing import AsyncIterator, Optional, List

import aiohttp

from inputs.base.loop import LoopInput


class TwitterInput(LoopInput[str]):
    """Context query input handler for RAG"""
    
    def __init__(self, api_url: str = "https://api.openmind.org/api/core/query", config: dict = None):
        super().__init__()
        self.buffer: List[str] = []
        self.message_buffer: Queue[str] = Queue()
        self.api_url = api_url
        self.session: Optional[aiohttp.ClientSession] = None
        self.context: Optional[str] = None
        self.config = config or {}
        
        # Store query from config if provided
        self.query = self.config.get('query', "What's new in AI and technology?")

    async def __aenter__(self):
        """Async context manager entry"""
        await self._init_session()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit"""
        if self.session:
            await self.session.close()

    async def _init_session(self):
        """Initialize aiohttp session if not exists"""
        if self.session is None:
            timeout = aiohttp.ClientTimeout(total=10)
            self.session = aiohttp.ClientSession(timeout=timeout)

    async def _query_context(self, query: str):
        """Perform context query to RAG endpoint"""
        await self._init_session()
        
        try:
            async with self.session.post(
                self.api_url,
                json={"query": query},
                headers={"Content-Type": "application/json"}
            ) as response:
                if response.status == 200:
                    data = await response.json()
                    if "results" in data:
                        documents = data["results"]
                        context = '\n\n'.join([
                            r.get('content', {}).get('text', '') 
                            for r in documents 
                            if r.get('content', {}).get('text', '')
                        ])
                        self.context = context
                        self.buffer = [context]  # Replace buffer with context
                else:
                    error_text = await response.text()
                    logging.error(f"Query failed with status {response.status}: {error_text}")
                    
        except Exception as e:
            logging.error(f"Error querying context: {str(e)}")

    async def raw_to_text(self, raw_input):
        """Convert raw input to processed text and manage buffer"""
        logging.debug(f"raw_to_text received: {raw_input}")
        text = await self._raw_to_text(raw_input)
        logging.debug(f"_raw_to_text returned: {text}")
        
        if text is not None:
            logging.debug(f"Adding to buffer: {text}")
            # If we have text but no context, treat it as a query
            if not self.context:
                await self._query_context(text)
            logging.debug(f"Buffer now contains: {self.buffer}")

    async def _raw_to_text(self, raw_input: str) -> Optional[str]:
        """Convert raw input to text format and add to buffer"""
        if raw_input:
            self.message_buffer.put_nowait(raw_input)  # Add to message buffer
        return raw_input

    async def start(self):
        """Start the input handler with initial query"""
        await self._query_context(self.query)
        self.message_buffer.put_nowait(self.query)

    async def listen(self) -> AsyncIterator[str]:
        """Listen for new messages"""
        await self.start()
        
        while True:
            message = await self._poll()
            if message:
                yield message
            await asyncio.sleep(0.1)

    async def _poll(self) -> Optional[str]:
        """Poll for new messages"""
        await asyncio.sleep(0.5)
        try:
            message = self.message_buffer.get_nowait()
            return message
        except Empty:
            return None

    def formatted_latest_buffer(self) -> Optional[str]:
        """Format and return the context"""
        content = self.context if self.context else (self.buffer[-1] if self.buffer else None)
        
        if not content:
            return None

        result = f"""
TwitterInput CONTEXT
// START
{content}
// END
"""
        return result

    async def initialize_with_query(self, query: str):
        """Initialize with a query"""
        logging.info(f"[TwitterInput] Initializing with query: {query}")
        self.message_buffer.put_nowait(query)  # Add query to message buffer
        await self._query_context(query)  # Immediately get context 