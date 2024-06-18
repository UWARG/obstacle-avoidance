"""
Test for flight interface by printing to device.
"""

from modules.flight_interface import flight_interface


def create_flight_interface_instance(
    address: str, timeout: float
) -> "tuple[bool, flight_interface.FlightInterface | None]":
    """
    Construct a flight interface instance.
    """
    result, flight_interface_instance = flight_interface.FlightInterface.create(address, timeout)

    return result, flight_interface_instance


class TestFlightInterface:
    """
    Flight interface tests.
    """

    MISSION_PLANNER_ADDRESS = "tcp:127.0.0.1:14550"
    TIMEOUT = 1.0

    def test_create_invalid_address(self) -> None:
        """
        Test create method using an invalid Mission Planner IP address.
        """
        expected_result = False
        expected_instance = None
        actual_result, actual_instance = create_flight_interface_instance(
            "tcp:127.0.0.1:3000", self.TIMEOUT
        )
        assert actual_result == expected_result
        assert actual_instance == expected_instance
