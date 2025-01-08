from dataclasses import dataclass
from abc import ABC, abstractmethod
import typing as T

IT = T.TypeVar("IT")
OT = T.TypeVar("OT")


@dataclass
class Interface(T.Generic[IT, OT]):
    """
    An interface for a module.
    """

    input: IT
    output: OT


class ModuleImpl(ABC, T.Generic[IT, OT]):
    @abstractmethod
    async def execute(self, input_protocol: IT) -> OT:
        pass


class ModuleMutation(ABC, T.Generic[OT]):
    @abstractmethod
    async def mutate(self, input_protocol: OT) -> None:
        pass


@dataclass
class Module:
    name: str
    interface: T.Type[Interface]
    impl: ModuleImpl
    mutation: ModuleMutation
