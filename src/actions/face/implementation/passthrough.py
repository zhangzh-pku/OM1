from actions.base import ActionImplementation

from actions.face.interface import FaceInput


class FacePassthroughImplementation(ActionImplementation[FaceInput, FaceInput]):
    """
    A passthrough implementationementation of the face action. Output is the same as the input.
    """

    async def execute(self, input_interface: FaceInput) -> FaceInput:
        return input_interface
