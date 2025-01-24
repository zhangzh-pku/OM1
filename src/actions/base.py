import typing as T
from abc import ABC, abstractmethod
from dataclasses import dataclass

IT = T.TypeVar("IT")
OT = T.TypeVar("OT")


@dataclass
class Interface(T.Generic[IT, OT]):
    """
    An interface for a action.
    """

    input: IT
    output: OT


class ActionImplementation(ABC, T.Generic[IT, OT]):
    @abstractmethod
    async def execute(self, input_protocol: IT) -> OT:
        pass


class ActionConnector(ABC, T.Generic[OT]):
    @abstractmethod
    async def connect(self, input_protocol: OT) -> None:
        pass


@dataclass
class AgentAction:
    name: str
    interface: T.Type[Interface]
    implementation: ActionImplementation
    connector: ActionConnector
