"""
Test for flight interface by printing to device.
"""

import time

from modules.common.mavlink.modules import drone_odometry
from modules.flight_interface import flight_interface


def create_flight_interface_instance(
    address: str, timeout: float
) -> "tuple[bool, FlightInterface | None]":
    """
    Construct a flight interface instance.
    """
    result, flight_interface_instance = flight_interface.FlightInterface.create(address, timeout)

    return result, flight_interface_instance


def create_drone_position(
    latitude: float, longitude: float, altitude: float
) -> drone_odometry.DronePosition:
    """
    Contruct a drone position instance.
    """
    expected = True
    actual, global_position = drone_odometry.DronePosition.create(latitude, longitude, altitude)
    assert actual == expected
    assert global_position is not None

    return global_position


class TestFlightInterface:
    """
    Flight interface tests.
    """

    MISSION_PLANNER_ADDRESS = "tcp:127.0.0.1:14550"
    TIMEOUT = 1.0

    DELAY_TIME = 1.0

    def test_create_invalid_address(self) -> None:
        """
        Test create method using a valid Mission Planner IP address.
        """
        expected_result = False
        expected_instance = None
        actual_result, actual_instance = create_flight_interface_instance("", self.TIMEOUT)
        assert actual_result == expected_result
        assert actual_instance == expected_instance

    def test_create_valid_address(self) -> None:
        """
        Test create method using a valid Mission Planner IP address.
        """
        expected = True
        actual, instance = create_flight_interface_instance(
            self.MISSION_PLANNER_ADDRESS, self.TIMEOUT
        )
        assert actual == expected
        assert instance.controller is not None
        assert instance.home_location is not None

    def test_flight_interface(self) -> None:
        """
        Tests run function and prints results.
        """
        expected_result = True
        actual_result, flight_interface_instance = create_flight_interface_instance(
            self.MISSION_PLANNER_ADDRESS, self.TIMEOUT
        )

        assert actual_result == expected_result

        for _ in range(8):
            actual_result, local_odometry = flight_interface_instance.run()
            assert actual_result == expected_result
            assert local_odometry is not None

            time.sleep(self.DELAY_TIME)
