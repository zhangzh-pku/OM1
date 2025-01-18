from modules.base import ModuleImpl

from modules.face.interface import FaceInput


class FacePassthroughImpl(ModuleImpl[FaceInput, FaceInput]):
    """
    A passthrough implementation of the face module. Output is the same as the input.
    """

    async def execute(self, input_interface: FaceInput) -> FaceInput:
        return input_interface
