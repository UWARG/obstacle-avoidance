"""
Creates flight controller and produces local drone odometry coupled with a timestamp.
"""

from modules import decision_command
from modules import drone_odometry_local
from ..common.mavlink.modules import drone_odometry
from ..common.mavlink.modules import flight_controller
from . import conversions


class FlightInterface:
    """
    Create flight controller and sets home location.
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
        home_location: drone_odometry.DronePosition,
    ) -> None:
        """
        Private constructor, use create() method.
        """

        assert create_key is FlightInterface.__create_key, "Use create() method"

        self.controller = controller
        self.home_location = home_location

    def run(self) -> "tuple[bool, drone_odometry_local.DroneOdometryLocal | None]":
        """
        Returns local drone odometry with timestamp.
        """
        result, odometry = self.controller.get_odometry()
        if not result:
            return False, None

        global_position = odometry.position

        result, local_position = conversions.position_global_to_local(
            global_position, self.home_location
        )
        if not result:
            return False, None

        drone_orientation = odometry.orientation

        result, flight_mode = self.controller.get_flight_mode()
        if not result:
            return False, None

        flight_mode = drone_odometry_local.FlightMode(flight_mode.value)

        return drone_odometry_local.DroneOdometryLocal.create(
            local_position, drone_orientation, flight_mode
        )

    def run_decision_handler(self, command: decision_command.DecisionCommand) -> bool:
        """
        Uploads decision commands to drone.
        """
        if command.command == decision_command.DecisionCommand.CommandType.RESUME_MISSION:
            return self.resume_handler()
        if command.command == decision_command.DecisionCommand.CommandType.STOP_MISSION_AND_HALT:
            return self.stop_handler()
        return False

    def resume_handler(self) -> bool:
        """
        Resumes the AUTO mission.
        """
        result = self.controller.set_flight_mode("AUTO")
        if result:
            print("Flight interface: Successfully set flight mode to AUTO.")
        return result

    def stop_handler(self) -> bool:
        """
        Stops the drone.
        """
        result = self.controller.set_flight_mode("LOITER")
        if result:
            print("Flight interface: Successfully set flight mode to LOITER.")
        return result
