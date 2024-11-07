"""
Unit tests for lidar_parser module.
"""

import pytest

from modules import lidar_detection
from modules.lidar_parser import lidar_parser

ANGLE_UP = 15.0
ANGLE_DOWN = -15.0

# pylint: disable=redefined-outer-name, duplicate-code


@pytest.fixture()
def lidar_parser_instance() -> lidar_parser.LidarParser:  # type: ignore
    """
    Fixture to initialize a LidarParser instance for testing.
    """
    parser = lidar_parser.LidarParser()
    yield parser


@pytest.fixture()
def lidar_detection_up() -> lidar_detection.LidarDetection:  # type: ignore
    """
    Fixture to create an upward LidarDetection object.
    """
    result, detection = lidar_detection.LidarDetection.create(5.0, ANGLE_UP)
    assert result
    assert detection is not None
    yield detection


@pytest.fixture()
def higher_angle_detection_up(lidar_detection_up: lidar_detection.LidarDetection) -> lidar_detection.LidarDetection:  # type: ignore
    """
    Fixture to create a LidarDetection object with a higher upward angle than the previous detection.
    """
    result, higher_detection = lidar_detection.LidarDetection.create(
        5.0, lidar_detection_up.angle + 5.0
    )
    assert result
    assert higher_detection is not None
    yield higher_detection


@pytest.fixture()
def lidar_detection_down() -> lidar_detection.LidarDetection:  # type: ignore
    """
    Fixture to create a downward LidarDetection object.
    """
    result, detection = lidar_detection.LidarDetection.create(5.0, ANGLE_DOWN)
    assert result
    assert detection is not None
    yield detection


@pytest.fixture()
def lower_angle_detection_down(lidar_detection_up: lidar_detection.LidarDetection) -> lidar_detection.LidarDetection:  # type: ignore
    """
    Fixture to create a LidarDetection object with a lower downward angle than the previous detection.
    """
    result, lower_detection = lidar_detection.LidarDetection.create(
        5.0, lidar_detection_up.angle - 5.0
    )
    assert result
    assert lower_detection is not None
    yield lower_detection


class TestLidarParser:
    """
    Tests for the LidarParser run() method.
    """

    def test_initial_run_no_oscillation(
        self,
        lidar_parser_instance: lidar_parser.LidarParser,
        lidar_detection_up: lidar_detection.LidarDetection,
    ) -> None:
        """
        Test that a single upward detection does not complete an oscillation.
        """
        result, oscillation = lidar_parser_instance.run(lidar_detection_up)
        assert not result
        assert oscillation is None

    def test_oscillation_detected_up_to_down(
        self,
        lidar_parser_instance: lidar_parser.LidarParser,
        lidar_detection_up: lidar_detection.LidarDetection,
        higher_angle_detection_up: lidar_detection.LidarDetection,
        lidar_detection_down: lidar_detection.LidarDetection,
    ) -> None:
        """
        Test detection of an oscillation with a change in direction from up to down after multiple detections.
        """
        lidar_parser_instance.run(lidar_detection_up)
        lidar_parser_instance.run(higher_angle_detection_up)
        result, oscillation = lidar_parser_instance.run(lidar_detection_down)
        assert result
        assert oscillation is not None

    def test_no_oscillation_on_same_direction(
        self,
        lidar_parser_instance: lidar_parser.LidarParser,
        lidar_detection_up: lidar_detection.LidarDetection,
    ) -> None:
        """
        Test that no oscillation is detected when multiple detections are in the same upward direction.
        """
        for _ in range(5):
            result, oscillation = lidar_parser_instance.run(lidar_detection_up)
            assert not result
            assert oscillation is None

    def test_direction_initialization_on_first_run(
        self,
        lidar_parser_instance: lidar_parser.LidarParser,
        lidar_detection_up: lidar_detection.LidarDetection,
        lower_angle_detection_down: lidar_detection.LidarDetection,
    ) -> None:
        """
        Test that the initial detection sets the direction to DOWN based on a second downward detection.
        """
        lidar_parser_instance.run(lidar_detection_up)
        lidar_parser_instance.run(lower_angle_detection_down)
        assert lidar_parser_instance.direction == lidar_parser.Direction.DOWN

    def test_oscillation_reset_after_detection(
        self,
        lidar_parser_instance: lidar_parser.LidarParser,
        lidar_detection_up: lidar_detection.LidarDetection,
        higher_angle_detection_up: lidar_detection.LidarDetection,
        lidar_detection_down: lidar_detection.LidarDetection,
    ) -> None:
        """
        Test that the parser resets its readings after detecting an oscillation.
        """
        lidar_parser_instance.run(lidar_detection_up)
        lidar_parser_instance.run(higher_angle_detection_up)
        result, oscillation = lidar_parser_instance.run(lidar_detection_down)
        assert result
        assert oscillation is not None
        assert len(lidar_parser_instance.lidar_readings) == 0

    def test_alternating_up_down_oscillations(
        self,
        lidar_parser_instance: lidar_parser.LidarParser,
        lidar_detection_up: lidar_detection.LidarDetection,
        higher_angle_detection_up: lidar_detection.LidarDetection,
        lidar_detection_down: lidar_detection.LidarDetection,
    ) -> None:
        """
        Test the parser with alternating up and down angles to simulate multiple oscillations.
        """
        oscillation_count = 0
        lidar_parser_instance.run(lidar_detection_up)
        lidar_parser_instance.run(higher_angle_detection_up)
        result, oscillation = lidar_parser_instance.run(lidar_detection_down)

        if result:
            oscillation_count += 1
            assert oscillation is not None

        for i in range(4):
            if i % 2 == 0:
                result, oscillation = lidar_parser_instance.run(higher_angle_detection_up)
            else:
                result, oscillation = lidar_parser_instance.run(lidar_detection_down)

            if result:
                oscillation_count += 1
                assert oscillation is not None

        assert oscillation_count == 5

    def test_no_oscillation_with_single_reading(
        self,
        lidar_parser_instance: lidar_parser.LidarParser,
        lidar_detection_up: lidar_detection.LidarDetection,
    ) -> None:
        """
        Test that a single detection does not produce an oscillation.
        """
        result, oscillation = lidar_parser_instance.run(lidar_detection_up)
        assert not result
        assert oscillation is None

    def test_oscillation_on_direction_change_after_none(
        self,
        lidar_parser_instance: lidar_parser.LidarParser,
        lidar_detection_up: lidar_detection.LidarDetection,
        higher_angle_detection_up: lidar_detection.LidarDetection,
        lidar_detection_down: lidar_detection.LidarDetection,
    ) -> None:
        """
        Test that direction changes after Direction.NONE trigger the first oscillation properly.
        """
        lidar_parser_instance.run(lidar_detection_up)
        lidar_parser_instance.run(higher_angle_detection_up)
        assert lidar_parser_instance.direction == lidar_parser.Direction.UP
        result, oscillation = lidar_parser_instance.run(lidar_detection_down)
        assert result
        assert oscillation is not None
