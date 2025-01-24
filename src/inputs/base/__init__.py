import typing as T

R = T.TypeVar("R")


class AgentInput(T.Generic[R]):
    """
    Base class for all agent inputs.
    """

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
        raise NotImplementedError

    def formatted_latest_buffer(self) -> str | None:
        """
        Return the latest buffer formatted as a prompt string
        """
        raise NotImplementedError

    def latest_buffer(self) -> str | None:
        """
        Return the latest buffer string
        """
        raise NotImplementedError

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
