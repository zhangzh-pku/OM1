import logging

from modules.base import ModuleMutation
from modules.face.interface import FaceInput

class FaceRos2Mutation(ModuleMutation[FaceInput]):
    async def mutate(self, output_interface: FaceInput) -> None:

        # define/clarify the datatype
        new_msg = {"face": ""}

        if output_interface.action == "smile":
            new_msg["face"] = "smile"
        elif output_interface.action == "frown":
            new_msg["face"] = "frown"
        elif output_interface.action == "cry":
            new_msg["face"] = "cry"
        else:
            logging.info(f"Unknown face type: {output_interface.action}")
            # raise ValueError(f"Unknown face type: {output_interface.action}")

        logging.info(f"SendThisToROS2: {new_msg}")
