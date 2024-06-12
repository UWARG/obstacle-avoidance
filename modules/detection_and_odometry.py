"""
LiDAR detection and local odometry data structure.
"""

from . import drone_odometry_local
from . import lidar_detection


class DetectionAndOdometry:
    """
    Contains LiDAR reading and current local odometry.
    """

    __create_key = object()

    @classmethod
    def create(
        cls,
        detection: lidar_detection.LidarDetection,
        local_odometry: drone_odometry_local.DroneOdometryLocal,
    ) -> "tuple[bool, DetectionAndOdometry | None]":
        """
        Combines lidar reading with local odometry
        """

        if detection is None:
            return False, None

        if local_odometry is None:
            return False, None

        return True, DetectionAndOdometry(cls.__create_key, detection, local_odometry)

    def __init__(
        self,
        create_key: object,
        detection: lidar_detection.LidarDetection,
        local_odometry: drone_odometry_local.DroneOdometryLocal,
    ) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is DetectionAndOdometry.__create_key, "Use create() method"

        self.detection = detection
        self.odometry = local_odometry

    def __str__(self) -> str:
        """
        String representation.
        """
        return f"{self.__class__.__name__}, {self.detection}, str{self.odometry}"
