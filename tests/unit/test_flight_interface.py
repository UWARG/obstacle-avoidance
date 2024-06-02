"""
Test for flight interface by printing to device.
"""

import time
import pytest


from modules.flight_interface import flight_interface

from ..common.mavlink.modules import drone_odometry


@pytest.fixture()
def create_flight_interface_instance(
    address: str, timeout: float
) -> "tuple[bool, FlightInterface | None]":
    """
    Construct a flight interface instance.
    """
    result, flight_interface_instance = flight_interface.FlightInterface.create(address, timeout)

    yield result, flight_interface_instance


@pytest.fixture()
def create_drone_position(
    latitude: float, longitude: float, altitude: float
) -> drone_odometry.DronePosition:
    """
    Contruct a drone position instance.
    """
    result, global_position = drone_odometry.DronePosition.create(latitude, longitude, altitude)
    assert result
    assert global_position is not None

    yield global_position


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
        result, instance = create_flight_interface_instance(None, self.TIMEOUT)
        assert not result
        assert instance is None

    def test_create_valid_address(self) -> None:
        """
        Test create method using a valid Mission Planner IP address.
        """
        result, instance = create_flight_interface_instance(
            self.MISSION_PLANNER_ADDRESS, self.TIMEOUT
        )
        assert result
        assert instance.controller is not None
        assert instance.home_location is not None

    def test_flight_interface(self) -> None:
        """
        Tests run function and prints results.
        """

        result, flight_interface_instance = create_flight_interface_instance(
            self.MISSION_PLANNER_ADDRESS, self.TIMEOUT
        )

        for _ in range(8):
            result, local_odometry = flight_interface_instance.run()
            assert result
            assert local_odometry is not None

            print("north: " + str(local_odometry.local_position.north))
            print("east: " + str(local_odometry.local_position.east))
            print("down: " + str(local_odometry.local_position.down))
            print("roll: " + str(local_odometry.drone_orientation.roll))
            print("pitch: " + str(local_odometry.drone_orientation.pitch))
            print("yaw: " + str(local_odometry.drone_orientation.yaw))
            print("timestamp: " + str(local_odometry.timestamp))
            print("")

            time.sleep(self.DELAY_TIME)
