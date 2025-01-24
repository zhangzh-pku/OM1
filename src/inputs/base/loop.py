import typing as T

from inputs.base import AgentInput

R = T.TypeVar("R")


class LoopInput(AgentInput[R]):
    """
    Input polling in a continuous async loop
    """

    async def _listen_loop(self) -> T.AsyncIterator[R]:
        while True:
            yield await self._poll()

    async def _poll(self) -> R:
        """
        Poll for input
        """
        raise NotImplementedError
