import typing as T

R = T.TypeVar("R")


class AgentInput(T.Generic[R]):
    """
    Base class for all agent inputs.
    """

    buffer: list[str] = []

    async def listen(self) -> T.AsyncIterator[R]:
        """
        Asyncronous loop to listen for input events and yield results
        """
        async for event in self._listen_loop():
            yield event

    async def raw_to_text(self, raw_input: R) -> str:
        """
        Convert raw input to text for the fuser
        """
        text = await self._raw_to_text(raw_input)
        self.buffer.append(text)
        return text

    def formatted_latest_buffer(self) -> str | None:
        """
        Return the latest buffer formatted as a prompt string
        """
        if len(self.buffer) == 0:
            return None
        return f"""
                {self.__class__.__name__} INPUT
                // START
                {self.buffer[-1]}
                // END
                """

    def latest_buffer(self) -> str | None:
        """
        Return the latest buffer string
        """
        if len(self.buffer) == 0:
            return None
        return self.buffer[-1]

    async def _listen_loop(self) -> T.AsyncIterator[R]:
        """
        Asyncronous loop to listen for input events and yield results
        """
        raise NotImplementedError

    async def _raw_to_text(self, raw_input: R) -> str:
        """
        Convert raw input to text for the fuser
        """
        raise NotImplementedError
