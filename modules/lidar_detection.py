"""
LiDAR detection data structure.
"""


class LidarDetection:
    """
    Lidar scan.
    """

    __create_key = object()

    @classmethod
    def create(cls, distance: float, angle: float) -> "tuple[bool, LidarDetection | None]":
        """
        distance is in metres.
        angle is in degrees.
        """
        if distance == -1:
            return False, None
        return True, LidarDetection(cls.__create_key, distance, angle)

    def __init__(self, create_key: object, distance: float, angle: float) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is LidarDetection.__create_key, "Use create() method"

        self.distance = distance
        self.angle = angle

    def __str__(self) -> str:
        """
        String representation.
        """
        return f"{self.__class__.__name__}: distance: {self.distance}, angle: {self.angle}. "
