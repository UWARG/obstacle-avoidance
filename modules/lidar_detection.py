"""
LiDAR detection data structure.
"""


class LidarDetection:
    """
    Lidar scan
    """

    __create_key = object()

    __DISTANCE_LIMIT = 50
    __ANGLE_LIMIT = 170

    @classmethod
    def create(cls, distance: float, angle: float) -> "tuple[bool, LidarDetection | None]":
        """
        Distance is in meters.
        Angle is in degrees.
        """
        if distance < 0 or distance > cls.__DISTANCE_LIMIT:
            return False, None

        if abs(angle) > cls.__ANGLE_LIMIT:
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
        String representation
        """
        return f"{self.__class__.__name__}: Distance: {self.distance}, Angle: {self.angle}. "
