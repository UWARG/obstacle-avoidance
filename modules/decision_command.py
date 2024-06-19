"""
Command data structure used by decision module.
"""


class DecisionCommand:
    """
    Contains list of commands to send to drone.
    """

    __create_key = object()

    @classmethod
    def create(cls, command: "str") -> "tuple[bool, DecisionCommand | None]":
        """
        Creates a list of commands to send to the drone.
        """
        if command is None:
            return False, None
        return True, DecisionCommand(cls.__create_key, command)

    def __init__(self, create_key: object, command: str) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is DecisionCommand.__create_key, "Use create() method"

        self.command = command

    def __str__(self):
        pass
