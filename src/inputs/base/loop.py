import typing as T

from inputs.base import AgentInput

R = T.TypeVar("R")


class LoopInput(AgentInput[R]):
    """
    Input polling implementation using a continuous asynchronous loop.

    Repeatedly polls for input events in an async loop, yielding results
    as they become available.
    """

    def __init__(self):
        """
        Initialize LoopInput instance.
        """
        super().__init__()

    async def _listen_loop(self) -> T.AsyncIterator[R]:
        """
        Main polling loop that continuously yields input events.

        Yields
        ------
        R
            Raw input events from polling
        """
        while True:
            yield await self._poll()

    async def _poll(self) -> R:
        """
        Poll for next input event.

        Returns
        -------
        R
            Raw input event

        Raises
        ------
        NotImplementedError
            Must be implemented by subclasses
        """
        raise NotImplementedError
