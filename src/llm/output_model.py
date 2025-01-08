from pydantic import BaseModel, Field
import typing as T


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
