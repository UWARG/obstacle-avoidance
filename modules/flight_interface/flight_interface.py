
from . import conversions # to be implemented
from .. import drone_odometry_local

from ..common.mavlink.modules import drone_odometry
from ..common.mavlink.modules import flight_controller

class FlightInterface:

    __create_key = object()

    def create(
        cls,
        address: str,
        timeout_home: float
    ) -> "tuple[bool, FlightInterface | None]":
        result, controller = flight_controller.FlightController.create(address)
        if not result:
            return False, None
        
        result, home_location = controller.get_home_location(timeout_home)
        if not result:
            return False, None
        
        return True, FlightInterface(cls.__create_key, controller, home_location)

    def __init__(
        self, 
        create_key: object,
        controller: flight_controller.FlightController,
        home_location: drone_odometry.DroneLocation, 
    ) -> None:
        assert create_key is FlightInterface.__create_key, "Use create() method"

        self.controller = controller
        self.home_location = home_location

    def run(self) -> "tuple[bool, drone_odometry_local.DroneOdometryLocal | None]":

        result, drone_odometry = self.controller.get_odometry()
        if not result:
            return False, None

        drone_position = drone_odometry.position
        
        result, drone_position_local = conversions.global_to_local(drone_position, self.home_location)
        if not result:
            return False, None

        drone_orientation = drone_odometry.orientation

        return drone_odometry_local.DroneOdometryLocal.create(drone_position_local, drone_orientation)
    