from .. import detection_cluster
from .. import obstacle
from . import calculations


class ClusterClassification:
    """
    Classifies each cluster into a type of obstacle.
    """

    def __init__(self, circle_threshold: int, line_threshold: float) -> None:
        """
        circle_threshold: how many lidar readings in a cluster to be considered a circle obstacle.
        line_threshold: how close each point needs to be to the feature line to be considered a line obstacle.
        """
        self.circle_threshold = circle_threshold
        self.line_threshold = line_threshold

    def create_circle_obstacle(
        self, cluster: detection_cluster.DetectionCluster
    ) -> "tuple[bool, obstacle.Obstacle.Circle | None]":
        """
        Determines the centre and radius of the circle obstacle.
        """
        centre = calculations.midpoint_of_line(cluster[0], cluster[-1])
        radius = 0
        for point in cluster:
            distance = calculations.distance_between_two_points(point, centre)
            if distance > radius:
                radius = distance

        return obstacle.Obstacle.create_circle_obstacle(centre, radius)

    def create_line_obstacle(
        self, cluster: detection_cluster.DetectionCluster
    ) -> "tuple[bool, obstacle.Obstacle.Line | None]":
        """
        Determines the endpoints of the line obstacle.
        """
        start_point = cluster[0]
        end_point = cluster[-1]
        return obstacle.Obstacle.create_line_obstacle(start_point, end_point)

    def create_rect_obstacle(
        self, cluster: detection_cluster.DetectionCluster
    ) -> "tuple[bool, obstacle.Obstacle.Rect | None]":
        """
        Determines the vertices of the rect obstacle.
        """

        return obstacle.Obstacle.create_rect_obstacle(
            top_left, top_right, bottom_left, bottom_right
        )

    def run(self, cluster):
        """
        Classifies each cluster as a circle, line, or rect.
        """
        if cluster.cluster.empty():
            return False, None

        if len(cluster.cluster) <= self.circle_threshold:
            return self.create_circle_obstacle(cluster.cluster)

        start_point = cluster.cluster[0]
        end_point = cluster.cluster[-1]

        feature_line_length = calculations.length_of_line(start_point, end_point)
        threshold_distance = feature_line_length * 0.05

        for point in cluster.cluster:
            distance = calculations.distance_from_point_to_line(
                point,
                start_point,
                end_point,
            )
            if distance > threshold_distance:
                return self.create_rect_obstacle(cluster)

        return self.create_line_obstacle(cluster)
