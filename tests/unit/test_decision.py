"""
Test for decision module.
"""

import pytest

from modules import decision_command
from modules import detections_and_odometry
from modules import drone_odometry_local
from modules import lidar_detection
from modules.common.mavlink.modules import drone_odometry
from modules.common.mavlink.modules import flight_controller
from modules.decision import decision


OBJECT_PROXIMITY_LIMIT = 5.0  # metres
MAX_HISTORY = 20  # readings

# pylint: disable=redefined-outer-name, duplicate-code


@pytest.fixture()
def decision_maker() -> decision.Decision:  # type: ignore
    """
    Construct a decision instance with predefined object proximity limit.
    """
    decision_instance = decision.Decision(OBJECT_PROXIMITY_LIMIT, MAX_HISTORY)
    yield decision_instance


@pytest.fixture()
def object_within_proximity_limit_while_moving() -> detections_and_odometry.DetectionsAndOdometry:  # type: ignore
    """
    Creates a DetectionsAndOdometry instance within the proximity limit.
    """
    detections = []

    for _ in range(0, 5):
        result, detection = lidar_detection.LidarDetection.create(6.0, 3.0)
        assert result
        assert detection is not None
        detections.append(detection)

    result, detection = lidar_detection.LidarDetection.create(4.8, 3.0)
    assert result
    assert detection is not None
    detections.append(detection)

    result, position = drone_odometry_local.DronePositionLocal.create(0.0, 0.0, 0.0)
    assert result
    assert position is not None

    result, orientation = drone_odometry.DroneOrientation.create(0.0, 0.0, 0.0)
    assert result
    assert orientation is not None

    flight_mode = flight_controller.FlightController.FlightMode.MOVING

    result, odometry = drone_odometry_local.DroneOdometryLocal.create(
        position, orientation, flight_mode
    )
    assert result
    assert odometry is not None

    result, merged = detections_and_odometry.DetectionsAndOdometry.create(detections, odometry)
    assert result
    assert odometry is not None

    yield merged


@pytest.fixture()
def object_within_proximity_limit_while_stopped() -> detections_and_odometry.DetectionsAndOdometry:  # type: ignore
    """
    Creates a DetectionsAndOdometry instance within the proximity limit.
    """
    detections = []

    for _ in range(0, 5):
        result, detection = lidar_detection.LidarDetection.create(6.0, 3.0)
        assert result
        assert detection is not None
        detections.append(detection)

    result, detection = lidar_detection.LidarDetection.create(4.8, 3.0)
    assert result
    assert detection is not None
    detections.append(detection)

    result, position = drone_odometry_local.DronePositionLocal.create(0.0, 0.0, 0.0)
    assert result
    assert position is not None

    result, orientation = drone_odometry.DroneOrientation.create(0.0, 0.0, 0.0)
    assert result
    assert orientation is not None

    flight_mode = flight_controller.FlightController.FlightMode.STOPPED

    result, odometry = drone_odometry_local.DroneOdometryLocal.create(
        position, orientation, flight_mode
    )
    assert result
    assert odometry is not None

    result, merged = detections_and_odometry.DetectionsAndOdometry.create(detections, odometry)
    assert result
    assert odometry is not None

    yield merged


@pytest.fixture()
def object_outside_proximity_limit_while_moving() -> detections_and_odometry.DetectionsAndOdometry:  # type: ignore
    """
    Creates a DetectionsAndOdometry outside the proximity limit.
    """
    detections = []
    for _ in range(0, 5):
        result, detection = lidar_detection.LidarDetection.create(6.0, 3.0)
        assert result
        assert detection is not None
        detections.append(detection)

    result, position = drone_odometry_local.DronePositionLocal.create(0.0, 0.0, 0.0)
    assert result
    assert position is not None

    result, orientation = drone_odometry.DroneOrientation.create(0.0, 0.0, 0.0)
    assert result
    assert orientation is not None

    flight_mode = flight_controller.FlightController.FlightMode.MOVING

    result, odometry = drone_odometry_local.DroneOdometryLocal.create(
        position, orientation, flight_mode
    )
    assert result
    assert odometry is not None

    result, merged = detections_and_odometry.DetectionsAndOdometry.create(detections, odometry)
    assert result
    assert odometry is not None

    yield merged


@pytest.fixture()
def object_outside_proximity_limit_while_stopped() -> detections_and_odometry.DetectionsAndOdometry:  # type: ignore
    """
    Creates a DetectionsAndOdometry outside the proximity limit.
    """
    detections = []
    for _ in range(0, 5):
        result, detection = lidar_detection.LidarDetection.create(6.0, 3.0)
        assert result
        assert detection is not None
        detections.append(detection)

    result, position = drone_odometry_local.DronePositionLocal.create(0.0, 0.0, 0.0)
    assert result
    assert position is not None

    result, orientation = drone_odometry.DroneOrientation.create(0.0, 0.0, 0.0)
    assert result
    assert orientation is not None

    flight_mode = flight_controller.FlightController.FlightMode.STOPPED

    result, odometry = drone_odometry_local.DroneOdometryLocal.create(
        position, orientation, flight_mode
    )
    assert result
    assert odometry is not None

    result, merged = detections_and_odometry.DetectionsAndOdometry.create(detections, odometry)
    assert result
    assert odometry is not None

    yield merged


class TestDecision:
    """
    Test for the Decision.run() method.
    """

    def test_decision_within_proximity_limit_while_moving(
        self,
        decision_maker: decision.Decision,
        object_within_proximity_limit_while_moving: detections_and_odometry.DetectionsAndOdometry,
    ) -> None:
        """
        Test decision module when there is an object within the proximity limit and the drone is moving.
        """
        expected = decision_command.DecisionCommand.CommandType.STOP_MISSION_AND_HALT

        result, command = decision_maker.run(object_within_proximity_limit_while_moving)

        assert result
        assert command is not None
        assert command.command == expected

    def test_decision_within_proximity_limit_while_stopped(
        self,
        decision_maker: decision.Decision,
        object_within_proximity_limit_while_stopped: detections_and_odometry.DetectionsAndOdometry,
    ) -> None:
        """
        Test decision module when there is an object within the proximity limit and the drone is stopped.
        """
        expected = None

        result, command = decision_maker.run(object_within_proximity_limit_while_stopped)

        assert not result
        assert command == expected

    def test_decision_outside_proximity_limit_while_moving(
        self,
        decision_maker: decision.Decision,
        object_outside_proximity_limit_while_moving: detections_and_odometry.DetectionsAndOdometry,
    ) -> None:
        """
        Test decision module when there is an object outside the proximity limit and the drone is moving.
        """
        expected = None

        result, command = decision_maker.run(object_outside_proximity_limit_while_moving)

        assert not result
        assert command == expected

    def test_decision_outside_proximity_limit_while_stopped(
        self,
        decision_maker: decision.Decision,
        object_outside_proximity_limit_while_stopped: detections_and_odometry.DetectionsAndOdometry,
    ) -> None:
        """
        Test decision module when there is an object outside the proximity limit and the drone is stopped.
        """
        expected = decision_command.DecisionCommand.CommandType.RESUME_MISSION

        result, command = decision_maker.run(object_outside_proximity_limit_while_stopped)

        assert result
        assert command is not None
        assert command.command == expected
