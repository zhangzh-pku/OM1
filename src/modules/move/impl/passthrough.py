from modules.base import ModuleImpl

from modules.move.interface import MoveInput


class MovePassthroughImpl(ModuleImpl[MoveInput, MoveInput]):
    """
    A passthrough implementation of the move module. Output is the same as the input.
    """

    async def execute(self, input_interface: MoveInput) -> MoveInput:
        return input_interface
