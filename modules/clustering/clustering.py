"""
Clusters together LiDAR detections.
"""

import math

from .. import detection_cluster
from .. import detection_point
from .. import lidar_detection


class Clustering:
    """
    Groups together LiDAR detections into clusters.
    """

    def __init__(self, max_cluster_distance: float):
        """
        Initialize max distance between points in the same cluster.
        """
        self.max_cluster_distance = max_cluster_distance

        self.__clockwise = False
        self.__last_point = None
        self.__last_angle = None
        self.cluster = []

    def __calculate_distance_between_two_points(
        self, p1: detection_point.DetectionPoint, p2: detection_point.DetectionPoint
    ) -> float:
        """
        Distance calculation between two points.
        """
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def run(
        self, detection: lidar_detection.LidarDetection
    ) -> "tuple[bool, detection_cluster.DetectionCluster | None]":
        """
        Returns a DetectionCluster consisting of LidarDetections.
        """
        # convert to x, y coordinates
        detection_angle_in_radians = detection.angle * math.pi / 180
        x = math.cos(detection_angle_in_radians) * detection.distance
        y = math.sin(detection_angle_in_radians) * detection.distance

        result, point = detection_point.DetectionPoint.create(x, y)
        if not result:
            return False, None

        if self.__last_point is None:
            self.__last_point = point
            self.__last_angle = detection.angle
            self.cluster.append(point)
            return False, None

        # if lidar direction changes, start a new cluster
        direction_switched = False
        current_direction = self.__clockwise
        if detection.angle < self.__last_angle:
            self.__clockwise = False
        elif detection.angle > self.__last_angle:
            self.__clockwise = True
        if current_direction != self.__clockwise:
            direction_switched = True
        self.__last_angle = detection.angle

        # check distance from last point
        distance_from_last_point = self.__calculate_distance_between_two_points(
            point, self.__last_point
        )
        self.__last_point = point

        # if far enough, send current cluster, initialize new one
        if distance_from_last_point > self.max_cluster_distance or direction_switched:
            new_cluster = detection_cluster.DetectionCluster.create(self.cluster)
            self.cluster = []
            self.cluster.append(point)
            return new_cluster

        # if close enough, cluster together
        self.cluster.append(point)
        return False, None
