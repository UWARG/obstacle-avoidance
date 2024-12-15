"""
LiDAR detection and local odometry data structure.
"""

from . import lidar_detection
from . import odometry_and_waypoint


class DetectionsAndOdometry:
    """
    Contains LiDAR readings and current local odometry.
    """

    __create_key = object()

    @classmethod
    def create(
        cls,
        detections: "list[lidar_detection.LidarDetection]",
        local_odometry: odometry_and_waypoint.OdometryAndWaypoint,
    ) -> "tuple[bool, DetectionsAndOdometry | None]":
        """
        Combines lidar readings with local odometry
        """
        if len(detections) == 0:
            return False, None

        if local_odometry is None:
            return False, None

        return True, DetectionsAndOdometry(cls.__create_key, detections, local_odometry)

    def __init__(
        self,
        create_key: object,
        detections: "list[lidar_detection.LidarDetection]",
        local_odometry: odometry_and_waypoint.OdometryAndWaypoint,
    ) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is DetectionsAndOdometry.__create_key, "Use create() method"

        self.detections = detections
        self.odometry = local_odometry

    def __str__(self) -> str:
        """
        String representation.
        """
        detections_str = ", ".join(str(detection) for detection in self.detections)
        return f"{self.__class__.__name__}, Detections ({len(self.detections)}): {detections_str}, str{self.odometry}"
