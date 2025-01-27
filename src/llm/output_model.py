from pydantic import BaseModel, Field


class CommandArgument(BaseModel):
    """
    Single argument for a command execution.

    Parameters
    ----------
    name : str
        Name identifier for the argument
    value : str
        Value to be used for the argument
    """

    name: str = Field(..., description="The name of the argument")
    value: str = Field(..., description="The value of the argument")


class Command(BaseModel):
    """
    Executable command with its arguments.

    Parameters
    ----------
    name : str
        Name of the command to execute
    arguments : list[CommandArgument]
        List of arguments required for command execution
    """

    name: str = Field(..., description="The name of the command to execute")
    arguments: list[CommandArgument] = Field(
        ..., description="Arguments for the command"
    )


class CortexOutputModel(BaseModel):
    """
    Output model for the Cortex LLM responses.

    Parameters
    ----------
    commands : list[Command]
        Sequence of commands to be executed
    """

    commands: list[Command] = Field(..., description="List of commands to execute")
