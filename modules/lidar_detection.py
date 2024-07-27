"""
LiDAR detection data structure.
"""


class LidarDetection:
    """
    Lidar scan containing point relative to drone's position.
    """

    __create_key = object()

    @classmethod
    def create(
        cls, distance: float, angle: float, x: float, y: float
    ) -> "tuple[bool, LidarDetection | None]":
        """
        distance is in metres.
        angle is in degrees.
        (x, y) are the coordinates of the LiDAR scan relative to the drone's position.
        x is in meters.
        y is in meters.
        """
        if distance == -1:
            return False, None
        return True, LidarDetection(cls.__create_key, distance, angle, x, y)

    def __init__(
        self, create_key: object, distance: float, angle: float, x: float, y: float
    ) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is LidarDetection.__create_key, "Use create() method"

        self.distance = distance
        self.angle = angle
        self.x = x
        self.y = y

    def __str__(self) -> str:
        """
        String representation.
        """
        return f"{self.__class__.__name__}: distance: {self.distance}, angle: {self.angle}, x: {self.x}, y: {self.y}. "
