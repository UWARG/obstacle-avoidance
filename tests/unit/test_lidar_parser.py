"""
Unit tests for lidar_parser module.
"""

import pytest
from modules import lidar_detection
from modules import lidar_oscillation
from modules.lidar_parser import lidar_parser

ANGLE_UP = 15.0
ANGLE_DOWN = -15.0


@pytest.fixture()
def lidar_parser_instance() -> lidar_parser.LidarParser:  # type: ignore
    parser = lidar_parser.LidarParser()
    yield parser


@pytest.fixture()
def lidar_detection_up() -> lidar_detection.LidarDetection:  # type: ignore
    result, detection = lidar_detection.LidarDetection.create(5.0, ANGLE_UP)
    assert result
    assert detection is not None
    yield detection


@pytest.fixture()
def higher_angle_detection_up(lidar_detection_up: lidar_detection.LidarDetection) -> lidar_detection.LidarDetection:  # type: ignore
    result, higher_detection = lidar_detection.LidarDetection.create(
        5.0, lidar_detection_up.angle + 5.0
    )
    assert result
    assert higher_detection is not None
    yield higher_detection


@pytest.fixture()
def lidar_detection_down() -> lidar_detection.LidarDetection:  # type: ignore
    result, detection = lidar_detection.LidarDetection.create(5.0, ANGLE_DOWN)
    assert result
    assert detection is not None
    yield detection


@pytest.fixture()
def lower_angle_detection_down(lidar_detection_up: lidar_detection.LidarDetection) -> lidar_detection.LidarDetection:  # type: ignore
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

        # Detect the first oscillation with three readings: two up, one down
        lidar_parser_instance.run(lidar_detection_up)
        lidar_parser_instance.run(higher_angle_detection_up)
        result, oscillation = lidar_parser_instance.run(lidar_detection_down)

        if result:
            oscillation_count += 1
            assert oscillation is not None

        # Subsequent oscillations should alternate and be detected on every change of angle
        for i in range(4):
            if i % 2 == 0:  # Every even index provides an upward reading
                result, oscillation = lidar_parser_instance.run(higher_angle_detection_up)
            else:  # Every odd index provides a downward reading
                result, oscillation = lidar_parser_instance.run(lidar_detection_down)

            if result:
                oscillation_count += 1
                assert oscillation is not None

        # Verify that five oscillations were detected in total
        assert oscillation_count == 5

    def test_no_oscillation_with_single_reading(
        self,
        lidar_parser_instance: lidar_parser.LidarParser,
        lidar_detection_up: lidar_detection.LidarDetection,
    ) -> None:
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
        lidar_parser_instance.run(lidar_detection_up)
        lidar_parser_instance.run(higher_angle_detection_up)
        assert lidar_parser_instance.direction == lidar_parser.Direction.UP
        result, oscillation = lidar_parser_instance.run(lidar_detection_down)
        assert result
        assert oscillation is not None
