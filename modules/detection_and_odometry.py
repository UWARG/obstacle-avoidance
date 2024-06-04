"""
LiDAR detection and local odometry data structure.
"""

from . import drone_odometry_local


class LidarDetection:
    """
    Lidar scan
    """

    __create_key = object()

    @classmethod
    def create(cls, distance: float, angle: float) -> "tuple[bool, LidarDetection | None]":
        """
        Distance is in meters.
        Angle is in degrees.
        """

        return True, LidarDetection(cls.__create_key, distance, angle)

    def __init__(self, create_key: object, distance: float, angle: float):
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
        return f"Distance: {self.distance}, Angle: {self.angle}. "


class DetectionAndOdometry:
    """
    Contains LiDAR reading and current local odometry.
    """

    __create_key = object()

    @classmethod
    def create(
        cls,
        lidar_detection: LidarDetection,
        local_odometry: drone_odometry_local.DroneOdometryLocal,
    ) -> "tuple[bool, DetectionAndOdometry | None]":
        """
        Combines lidar reading with local odometry
        """

        return True, DetectionAndOdometry(cls.__create_key, lidar_detection, local_odometry)

    def __init__(
        self,
        create_key: object,
        lidar_detection: LidarDetection,
        local_odometry: drone_odometry_local.DroneOdometryLocal,
    ) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is DetectionAndOdometry.__create_key, "Use create() method"

        self.detection = lidar_detection
        self.odometry = local_odometry

    def __str__(self) -> str:
        """
        String representation.
        """
        return f"{self.__class__.__name__}, {self.detection}, str{self.odometry}"
