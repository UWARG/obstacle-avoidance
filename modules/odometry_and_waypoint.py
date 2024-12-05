"""
Data structure for local odometry data (local position, orientation, 
flight mode, and timestamp) combined with next_waypoint for obstacle avoidance.
"""

import time
import enum
from modules.common.modules import position_local
from modules.common.modules import orientation


class FlightMode(enum.Enum):
    """
    Possible drone flight modes.
    """

    AUTO = "AUTO"
    LOITER = "LOITER"
    GUIDED = "GUIDED"
    RTL = "RTL"


class OdometryAndWaypoint:
    """
    Data structure combining odometry (local position, orientation, and flight mode),
    the next waypoint (local position), and a timestamp.
    """

    __create_key = object()

    @classmethod
    def create(
        cls,
        local_position: position_local.PositionLocal,
        drone_orientation: orientation.Orientation,
        flight_mode: FlightMode,
        next_waypoint: position_local.PositionLocal,
    ) -> "tuple[bool, OdometryAndWaypoint | None]":
        """
        Combines odometry data, next waypoint, and timestamp.
        """
        if local_position is None:
            return False, None

        if drone_orientation is None:
            return False, None

        if flight_mode is None:
            return False, None

        if next_waypoint is None:
            return False, None

        timestamp = time.time()

        return True, OdometryAndWaypoint(
            cls.__create_key,
            local_position,
            drone_orientation,
            flight_mode,
            next_waypoint,
            timestamp,
        )

    def __init__(
        self,
        create_key: object,
        local_position: position_local.PositionLocal,
        drone_orientation: orientation.Orientation,
        flight_mode: FlightMode,
        next_waypoint: position_local.PositionLocal,
        timestamp: float,
    ) -> None:
        """
        Private constructor, use create() method.
        """
        assert create_key is OdometryAndWaypoint.__create_key, "Use create() method"

        self.local_position = local_position
        self.drone_orientation = drone_orientation
        self.flight_mode = flight_mode
        self.next_waypoint = next_waypoint
        self.timestamp = timestamp

    def __str__(self) -> str:
        """
        String representation.
        """
        return (
            f"{self.__class__}, "
            f"Local Position: {self.local_position}, "
            f"Orientation: Yaw: {self.drone_orientation.yaw}, "
            f"Pitch: {self.drone_orientation.pitch}, "
            f"Roll: {self.drone_orientation.roll}, "
            f"Flight Mode: {self.flight_mode.name}, "
            f"Next Waypoint: {self.next_waypoint}, "
            f"Timestamp: {self.timestamp}"
        )
