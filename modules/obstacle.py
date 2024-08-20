"""
Obstacle data structure.
"""

from . import lidar_detection


class Obstacle:
    """
    Contains obstacles that represent lidar readings.

    The following are valid obstacle contructors:

    * Obstacle.create_circle_obstacle
    * Obstacle.create_line_obstacle
    * Obstacle.create_rect_obstacle
    """

    create_key = object()

    @classmethod
    def create_circle_obstacle(
        cls, centre: lidar_detection.LidarDetection, radius: float
    ) -> "tuple[bool, Obstacle.Circle | None]":
        """
        Circle obstacle contructor.
        Radius is in meters.
        Centre is represented by LidarDetection which has (x, y) NED coordinates relative to the drone's home position.
        """
        if radius <= 0:
            return False, None
        return True, Obstacle.Circle(cls.create_key, centre, radius)

    @classmethod
    def create_line_obstacle(
        cls, start_point: lidar_detection.LidarDetection, end_point: lidar_detection.LidarDetection
    ) -> "tuple[bool, Obstacle.Line | None]":
        """
        Line obstacle constructor.
        Endpoints are represented by LidarDetection which has (x, y) NED coordinates relative to the drone's home position.
        """
        return True, Obstacle.Line(cls.create_key, start_point, end_point)

    @classmethod
    def create_rect_obstacle(
        cls,
        top_left: lidar_detection.LidarDetection,
        top_right: lidar_detection.LidarDetection,
        bottom_left: lidar_detection.LidarDetection,
        bottom_right: lidar_detection.LidarDetection,
    ) -> "tuple[bool, Obstacle.Rect | None]":
        """
        Rect obstacle constructor.
        Corners are represented by LidarDetection which has (x, y) NED coordinates relative to the drone's home position.
        """
        if top_left.x > top_right.x:
            return False, None
        if top_left.y > bottom_left.y:
            return False, None
        if top_right.y > bottom_right.y:
            return False, None
        if bottom_left.x > bottom_right.x:
            return False, None
        return True, Obstacle.Rect(cls.create_key, top_left, top_right, bottom_left, bottom_right)

    class Circle:
        """
        Circle obstacle.
        """

        def __init__(
            self, create_key: object, centre: lidar_detection.LidarDetection, radius: float
        ) -> None:
            """
            Private constructor, use create() method.
            """
            assert create_key is Obstacle.create_key, "Use create() method"

            self.centre = centre
            self.radius = radius

    class Line:
        """
        Line obstacle.
        """

        def __init__(
            self,
            create_key: object,
            start_point: lidar_detection.LidarDetection,
            end_point: lidar_detection.LidarDetection,
        ) -> None:
            """
            Private constructor, use create() method.
            """
            assert create_key is Obstacle.create_key, "Use create() method"

            self.start_point = start_point
            self.end_point = end_point

    class Rect:
        """
        Rect obstacle.
        """

        def __init__(
            self,
            create_key: object,
            top_left: lidar_detection.LidarDetection,
            top_right: lidar_detection.LidarDetection,
            bottom_left: lidar_detection.LidarDetection,
            bottom_right: lidar_detection.LidarDetection,
        ) -> None:
            """
            Private constructor, use create() method.
            """
            assert create_key is Obstacle.create_key, "Use create() method"

            self.top_left = top_left
            self.top_right = top_right
            self.bottom_left = bottom_left
            self.bottom_right = bottom_right
