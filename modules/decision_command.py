"""
Command data structure used by decision module.
"""

import enum


class DecisionCommand:
    """
    Contains command to send to drone.

    The following are valid command constructors:

    * DecisionCommand.create_stop_command
    * DecisionCommand.create_resume_command
    """

    __create_key = object()

    class CommandType(enum.Enum):
        """
        Valid commands.
        """

        STOP_MISSION_AND_HALT = 0
        RESUME_MISSION = 1

    @classmethod
    def create_stop_command(cls) -> "tuple[bool, DecisionCommand | None]":
        """
        Command to stop and loiter the drone.
        """
        return True, DecisionCommand(cls.__create_key, DecisionCommand.CommandType.STOP_MISSION_AND_HALT)

    @classmethod
    def create_resume_command(cls) -> "tuple[bool, DecisionCommand | None]":
        """
        Command to resume auto mission.
        """
        return True, DecisionCommand(cls.__create_key, DecisionCommand.CommandType.RESUME_MISSION)

    def __init__(self, create_key: object, command: CommandType) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is DecisionCommand.__create_key, "Use create() method"

        self.command = command

    def __str__(self) -> str:
        """
        String representation
        """
        return f"{self.__class__.__name__}: {self.command}"
