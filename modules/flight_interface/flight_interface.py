from . import conversions
from .. import drone_odometry_local

from ..common.mavlink.modules import drone_odometry
from ..common.mavlink.modules import flight_controller


class FlightInterface:
    """
    Create flight controller and sets home location
    """

    __create_key = object()

    @classmethod
    def create(cls, address: str, timeout_home: float) -> "tuple[bool, FlightInterface | None]":
        """
        address: TCP address or port.
        timeout_home: Timeout for home location in seconds.
        """
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
        """
        Returns local drone odometry with timestamp
        """
        result, odometry = self.controller.get_odometry()
        if not result:
            return False, None

        global_position = odometry.position

        result, local_position = conversions.global_to_local(global_position, self.home_location)
        if not result:
            return False, None

        drone_orientation = odometry.orientation

        return drone_odometry_local.DroneOdometryLocal.create(local_position, drone_orientation)