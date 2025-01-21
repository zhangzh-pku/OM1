from dataclasses import dataclass
# from abc import ABC, abstractmethod
# import typing as T

# IT = T.TypeVar("IT")
# OT = T.TypeVar("OT")


# @dataclass
# class Interface(T.Generic[IT, OT]):
#     """
#     An interface for a action.
#     """

#     input: IT
#     output: OT


# class SimulatorImplementation(ABC, T.Generic[IT, OT]):
#     @abstractmethod
#     async def execute(self, input_protocol: IT) -> OT:
#         pass


# class SimulatorConnector(ABC, T.Generic[OT]):
#     @abstractmethod
#     async def connect(self, input_protocol: OT) -> None:
#         pass


@dataclass
class Simulator:
    name: str
    # interface: T.Type[Interface]
    # implementation: SimulatorImplementation
    # connector: SimulatorConnector
