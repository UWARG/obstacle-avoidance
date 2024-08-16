"""
Obstacles and local odometry merged data structure.
"""

from . import drone_odometry_local
from . import obstacle


class ObstaclesAndOdometry:
    """
    Contains obstacles and current local odometry.
    """

    __create_key = object()

    @classmethod
    def create(
        cls,
        obstacles: "list[obstacle.Obstacle]",
        local_odometry: drone_odometry_local.DroneOdometryLocal,
    ) -> "tuple[bool, ObstaclesAndOdometry | None]":
        """
        Combines obstacles with local odometry.
        """
        if len(obstacles) == 0:
            return False, None

        if local_odometry is None:
            return False, None

        return True, ObstaclesAndOdometry(cls.__create_key, obstacles, local_odometry)

    def __init__(
        self,
        create_key: object,
        obstacles: "list[obstacle.Obstacle]",
        local_odometry: drone_odometry_local.DroneOdometryLocal,
    ) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is ObstaclesAndOdometry.__create_key, "Use create() method"

        self.obstacles = obstacles
        self.odometry = local_odometry

    def __str__(self) -> str:
        """
        String representation.
        """
        obstacles_str = ", ".join(str(obstacle) for obstacle in self.obstacles)
        return f"{self.__class__.__name__}, Obstacles: ({len(self.obstacles)}): {obstacles_str}, str{self.odometry}"
