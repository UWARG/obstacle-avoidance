"""
Creates flight controller and produces local drone odometry coupled with a timestamp.
"""

import math

from modules import decision_command
from modules import drone_odometry_local
from ..common.mavlink.modules import drone_odometry
from ..common.mavlink.modules import flight_controller
from . import conversions


class FlightInterface:
    """
    Create flight controller and sets home location.
    To initialize flight interface, at least one waypoint must be set and written to the drone.
    """

    __create_key = object()

    @classmethod
    def create(
        cls, address: str, timeout_home: float, first_waypoint_distance_tolerance: float
    ) -> "tuple[bool, FlightInterface | None]":
        """
        address: TCP address or port.
        timeout_home: Timeout for home location in seconds.
        first_waypoint_distance_tolerance: flight interface can send commands after the drone has been within this
                                           set distance from the first waypoint (in metres).
        """
        result, controller = flight_controller.FlightController.create(address)
        if not result:
            return False, None

        result, home_location = controller.get_home_location(timeout_home)
        if not result:
            return False, None

        result, first_waypoint = controller.get_next_waypoint()
        if not result:
            print("Error initializing flight interface: check waypoints are loaded.")
            return False, None

        result, first_waypoint_local = conversions.position_global_to_local(
            first_waypoint, home_location
        )
        if not result:
            return False, None

        print("Flight interface successfully initialized.")
        print(f"Home location: {home_location}")
        print(f"First waypoint: {first_waypoint}")
        print(
            f"Obstacle avoidance will start {first_waypoint_distance_tolerance}m from first waypoint."
        )

        return True, FlightInterface(
            cls.__create_key,
            controller,
            home_location,
            first_waypoint_local,
            first_waypoint_distance_tolerance,
        )

    def __init__(
        self,
        create_key: object,
        controller: flight_controller.FlightController,
        home_location: drone_odometry.DronePosition,
        first_waypoint: drone_odometry_local.DronePositionLocal,
        first_waypoint_distance_tolerance: float,
    ) -> None:
        """
        Private constructor, use create() method.
        """

        assert create_key is FlightInterface.__create_key, "Use create() method"

        self.controller = controller
        self.home_location = home_location
        self.first_waypoint = first_waypoint
        self.first_waypoint_distance_tolerance = first_waypoint_distance_tolerance
        self.__run = False

    def __distance_to_first_waypoint_squared(
        self, local_position: drone_odometry_local.DronePositionLocal
    ) -> float:
        """
        Calculates the current distance in metres to the first waypoint (value is squared).
        """
        delta_x = local_position.north - self.first_waypoint.north
        delta_y = local_position.east - self.first_waypoint.east
        return delta_x**2 + delta_y**2

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

        if not self.__run:
            distance_to_first_waypoint_squared = self.__distance_to_first_waypoint_squared(
                local_position
            )
            print(f"Distance to first waypoint: {math.sqrt(distance_to_first_waypoint_squared)}m.")
            if distance_to_first_waypoint_squared <= self.first_waypoint_distance_tolerance**2:
                self.__run = True
                print("Obstacle avoidance started!")

        return drone_odometry_local.DroneOdometryLocal.create(
            local_position, drone_orientation, flight_mode, self.first_waypoint
        )

    def run_decision_handler(self, command: decision_command.DecisionCommand) -> bool:
        """
        Uploads decision commands to drone.
        """
        if not self.__run:
            return False
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
