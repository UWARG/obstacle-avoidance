"""
LiDAR detection data structure.
"""


class LidarDetection:
    """
    Lidar scan containing point relative to drone's position.
    """

    __create_key = object()

    @classmethod
    def create(cls, x: float, y: float) -> "tuple[bool, LidarDetection | None]":
        """
        x is in meters.
        y is in meters.
        """
        return True, LidarDetection(cls.__create_key, x, y)

    def __init__(self, create_key: object, x: float, y: float) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is LidarDetection.__create_key, "Use create() method"

        self.x = x
        self.y = y

    def __str__(self) -> str:
        """
        String representation.
        """
        return f"{self.__class__.__name__}: x: {self.x}, y: {self.y}. "
