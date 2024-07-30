"""
LiDAR detection local coordinates data structure.
"""


class DetectionPoint:
    """
    Local coordinates of a LiDAR detection.
    """

    __create_key = object()

    @classmethod
    def create(cls, x: float, y: float) -> "tuple[bool, DetectionPoint | None]":
        """
        (x, y) are the local coordinates of the LiDAR detection relative to the drone's position.
        x is the x-coordinate of the LiDAR detection in metres.
        y is the y-coordinate of the LiDAR detection in metres.
        """

        return True, DetectionPoint(cls.__create_key, x, y)

    def __init__(self, create_key: object, x: float, y: float) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is DetectionPoint.__create_key, "Use create() method"

        self.x = x
        self.y = y

    def __str__(self) -> str:
        """
        String representation.
        """
        return f"{self.__class__.__name__}: x: {self.x}, y: {self.y}. "
