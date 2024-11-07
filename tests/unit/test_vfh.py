"""
Unit tests for the VectorFieldHistogram module.
"""

import pytest

from modules.vfh import vfh
from modules import lidar_oscillation
from modules import lidar_detection
from modules import polar_obstacle_density

# Constants for default parameters
SECTOR_WIDTH = 2.0
MAX_VECTOR_MAGNITUDE = 1.0
LINEAR_DECAY_RATE = 0.05
CONFIDENCE_VALUE = 0.9
START_ANGLE = -90.0
END_ANGLE = 90.0

# pylint: disable=redefined-outer-name, duplicate-code


@pytest.fixture()
def vfh_default() -> vfh.VectorFieldHistogram:  # type: ignore
    """
    Creates a VectorFieldHistogram instance with default parameters.
    """
    return vfh.VectorFieldHistogram(
        sector_width=SECTOR_WIDTH,
        max_vector_magnitude=MAX_VECTOR_MAGNITUDE,
        linear_decay_rate=LINEAR_DECAY_RATE,
        confidence_value=CONFIDENCE_VALUE,
        start_angle=START_ANGLE,
        end_angle=END_ANGLE,
    )


@pytest.fixture()
def sample_lidar_oscillation() -> lidar_oscillation.LidarOscillation:  # type: ignore
    """
    Creates a sample LidarOscillation instance with predefined readings.
    """
    readings = []
    # Using the create() method to initialize LidarDetection instances
    result, detection = lidar_detection.LidarDetection.create(1.0, -45.0)
    readings.append(detection)

    result, detection = lidar_detection.LidarDetection.create(2.0, 0.0)
    readings.append(detection)

    result, detection = lidar_detection.LidarDetection.create(1.5, 45.0)
    readings.append(detection)

    # Continue initializing LidarOscillation with these readings
    result, oscillation = lidar_oscillation.LidarOscillation.create(readings)
    if result:
        return oscillation
    return None


@pytest.fixture()
def oscillation_with_object_in_quadrant() -> lidar_oscillation.LidarOscillation:  # type: ignore
    """
    Creates a LidarOscillation from -90 to +90 degrees with clear space everywhere
    except between 45 and 90 degrees where an object is detected at 20m.
    Clear space is signaled by 654.36m.
    """
    readings = []
    for angle in range(-90, 91, 1):
        distance = 10.0 if 45 <= angle <= 90 else 654.36
        result, reading = lidar_detection.LidarDetection.create(distance, angle)
        assert result
        readings.append(reading)

    result, oscillation = lidar_oscillation.LidarOscillation.create(readings)
    assert result
    yield oscillation


class TestVectorFieldHistogram:  # type: ignore
    """
    Tests for the VectorFieldHistogram.run() method.
    """

    def test_initialization_with_default_parameters(  # type: ignore
        self, vfh_default: vfh.VectorFieldHistogram
    ) -> None:
        """
        Test VectorFieldHistogram initialization with default parameters.
        """
        assert vfh_default.sector_width == SECTOR_WIDTH
        assert vfh_default.max_vector_magnitude == MAX_VECTOR_MAGNITUDE
        assert vfh_default.linear_decay_rate == LINEAR_DECAY_RATE
        assert vfh_default.confidence_value == CONFIDENCE_VALUE
        assert vfh_default.start_angle == START_ANGLE
        assert vfh_default.end_angle == END_ANGLE
        assert vfh_default.num_sectors == int((END_ANGLE - START_ANGLE) / SECTOR_WIDTH)

    def test_run_with_valid_lidar_oscillation(  # type: ignore
        self,
        vfh_default: vfh.VectorFieldHistogram,
        sample_lidar_oscillation: lidar_oscillation.LidarOscillation,
    ) -> None:
        """
        Test VectorFieldHistogram.run() method with a valid LidarOscillation input.
        """
        result, polar_density = vfh_default.run(sample_lidar_oscillation)
        assert result
        assert polar_density is not None
        assert isinstance(polar_density, polar_obstacle_density.PolarObstacleDensity)

    def test_run_with_object_in_quadrant(  # type: ignore
        self,
        vfh_default: vfh.VectorFieldHistogram,
        oscillation_with_object_in_quadrant: lidar_oscillation.LidarOscillation,
    ) -> None:
        """
        Test VectorFieldHistogram.run with an oscillation where an object is detected from 45 to 90 degrees.
        """
        result, polar_density = vfh_default.run(oscillation_with_object_in_quadrant)

        print(polar_density)

        assert result, "Expected valid result from VFH run."
        assert polar_density is not None, "Expected a non-null PolarObstacleDensity output."

        for sector in polar_density.sector_densities:
            if 44 <= sector.angle_start < 90:
                assert (
                    sector.density > 0
                ), f"Expected density > 0 in sector starting at {sector.angle_start} degrees."
            else:
                assert (
                    sector.density == 0
                ), f"Expected zero density in sector starting at {sector.angle_start} degrees."
