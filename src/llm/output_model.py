from pydantic import BaseModel, Field
from dataclasses import dataclass
import typing as T
import time

class CommandArgument(BaseModel):
    name: str = Field(..., description="The name of the argument")
    value: str = Field(..., description="The value of the argument")


class Command(BaseModel):
    name: str = Field(..., description="The name of the command to execute")
    arguments: list[CommandArgument] = Field(
        ..., description="Arguments for the command"
    )


class CortexOutputModel(BaseModel):
    """
    Output model for the Cortex LLM
    """
    commands: list[Command] = Field(..., description="List of commands to execute")

@dataclass
class LLM_full:
    """Class for a complete input / output cycle with timestamps."""
    prompt: str
    input_list: list[str]
    commands: list[Command]
    time_fuse: str
    time_submit: str # ms resolution timestamp
    time_done: str