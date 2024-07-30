"""
LiDAR detection cluster data structure.
"""

from . import lidar_detection


class DetectionCluster:
    """
    Cluster of LiDAR detections.
    """

    __create_key = object()

    @classmethod
    def create(
        cls, detections: "list[lidar_detection.LidarDetection]"
    ) -> "tuple[bool, DetectionCluster]":
        """
        Combines lidar readings in close proximity.
        """
        return True, DetectionCluster(cls.__create_key, detections)

    def __init__(
        self, create_key: object, detections: "list[lidar_detection.LidarDetection]"
    ) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is DetectionCluster.__create_key, "Use create() method"

        self.detections = detections

    def __str__(self) -> str:
        """
        String representation.
        """
        detections_str = ", ".join(str(detection) for detection in self.detections)
        return f"{self.__class__.__name__} (Count: {len(self.detections)}): {detections_str}. "
