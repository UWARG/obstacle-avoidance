"""
LiDAR detection and local odometry data structure.
"""

from . import drone_odometry_local


class DetectionAndOdometry:
    """
    Contains LiDAR reading and current local odometry.
    """

    __create_key = object()

    @classmethod
    def create(
        cls, distance: float, angle: float, local_odometry: drone_odometry_local.DroneOdometryLocal
    ) -> "tuple[bool, DetectionAndOdometry | None]":
        """
        Distance is in metres.
        Angle is in degrees.
        """

        return True, DetectionAndOdometry(cls.__create_key, distance, angle, local_odometry)

    def __init__(
        self,
        create_key: object,
        distance: float,
        angle: float,
        local_odometry: drone_odometry_local.DroneOdometryLocal,
    ) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is DetectionAndOdometry.__create_key, "Use create() method"

        self.distance = distance
        self.angle = angle
        self.odometry = local_odometry

    def __str__(self) -> str:
        """
        String representation.
        """
        return f"{self.__class__}, time: "
