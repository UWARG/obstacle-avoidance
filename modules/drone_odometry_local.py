"""
Data structure for local odometry data (local position, orientation, and timestamp)
"""

import time

from .common.mavlink.modules import drone_odometry



class DronePositionLocal:
    """
    Drone position using NED.
    """

    __create_key = object()

    @classmethod
    def create(
        cls, north: float, east: float, down: float
    ) -> "tuple[bool, DronePositionLocal | None]":
        """
        Local position (NED)
        """
        return True, DronePositionLocal(cls.__create_key, north, east, down)

    def __init__(self, create_key: object, north: float, east: float, down: float) -> None:
        """
        Private constructor, use create() method.
        """

        assert create_key is DronePositionLocal.__create_key, "Use create() method"

        self.north = north
        self.east = east
        self.down = down

    def __str__(self) -> str:
        """
        String representation
        """
        return f"{self.__class__}: North: {self.north}, East: {self.east}, Down: {self.down}."
    def __str__(self) -> str:
        """
        String representation
        """
        return f"{self.__class__}: North: {self.north}, East: {self.east}, Down: {self.down}."


class DroneOdometryLocal:
    """
    Data structure combining drone's local position, local orientation
    and timestamp.
    """

    __create_key = object()

    @classmethod
    def create(
        cls, local_position: DronePositionLocal, drone_orientation: drone_odometry.DroneOrientation
    ) -> "tuple[bool, DroneOdometryLocal | None]":
        """
        Combines local odometry data with timestamp
        """

        if local_position is None:
            return False, None

        if drone_orientation is None:
        cls, local_position: DronePositionLocal, drone_orientation: drone_odometry.DroneOrientation
    ) -> "tuple[bool, DroneOdometryLocal | None]":
        """
        Combines local odometry data with timestamp
        """

        if local_position is None:
            return False, None

        if drone_orientation is None:
            return False, None


        timestamp = time.time()

        return True, DroneOdometryLocal(
            cls.__create_key, local_position, drone_orientation, timestamp
        )
        return True, DroneOdometryLocal(
            cls.__create_key, local_position, drone_orientation, timestamp
        )

    def __init__(
        self,
        create_key: object,
        local_position: DronePositionLocal,
        drone_orientation: drone_odometry.DroneOrientation,
        timestamp: float,
    ) -> None:
        """
        Private constructor, use create() method.
        """

        assert create_key is DroneOdometryLocal.__create_key, "Use create() method"

        self.local_position = local_position
        self.local_position = local_position
        self.drone_orientation = drone_orientation
        self.timestamp = timestamp

    def __str__(self) -> str:
        """
        String representation
        """
        return f"{self.__class__},\
            {self.local_position}, \
                DroneOrientation: Roll: {self.drone_orientation.roll}, Pitch: {self.drone_orientation.pitch}, Yaw: {self.drone_orientation.yaw}.\
                    Time: {self.timestamp}."

    def __str__(self) -> str:
        """
        String representation
        """
        return f"{self.__class__},\
            {self.local_position}, \
                DroneOrientation: Roll: {self.drone_orientation.roll}, Pitch: {self.drone_orientation.pitch}, Yaw: {self.drone_orientation.yaw}.\
                    Time: {self.timestamp}."
