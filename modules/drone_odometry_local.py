
import time

from .common.mavlink.modules import drone_odometry

class DronePositionLocal:
    """
    Drone position using NED.
    """

    __create_key = object()

    @classmethod
    def create(
            cls, 
            north: float, 
            east: float, 
            down: float
        ) -> "tuple[bool, DronePositionLocal | None]":
        return True, DronePositionLocal(north, east, down)
    
    def __init__(
            self,
            create_key: object, 
            north: float, 
            east: float, 
            down: float
        ) -> None:
        
        assert create_key is DronePositionLocal.__create_key, "Use create() method"

        self.north = north
        self.east = east
        self.down = down



class DroneOdometryLocal:
    """
    Data structure combining drone's local position, local orientation
    and timestamp.
    """

    __create_key = object()

    @classmethod
    def create(
            cls, 
            drone_position: DronePositionLocal, 
            drone_orientation: drone_odometry.DroneOrientation
        ) -> "tuple[bool, DroneOdometryLocal | None]":
        
        if drone_position is None or drone_orientation is None:
            return False, None
        
        timestamp = time.time()

        return True, DroneOdometryLocal(drone_position, drone_orientation, timestamp)
    

    def __init__(
            self, 
            create_key: object,
            drone_position: DronePositionLocal, 
            drone_orientation: drone_odometry.DroneOrientation, 
            timestamp: float
            ) -> None:
        
        assert create_key is DroneOdometryLocal.__create_key, "Use create() method"

        self.drone_position = drone_position
        self.drone_orientation = drone_orientation
        self.timestamp = timestamp
