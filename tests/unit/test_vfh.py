"""
Unit tests for the VectorFieldHistogram module.
"""

import pytest
import numpy as np
from modules import vfh
from modules import lidar_oscillation
from modules import polar_obstacle_density

# Constants for default parameters
SECTOR_WIDTH = 2.0
MAX_VECTOR_MAGNITUDE = 1.0
LINEAR_DECAY_RATE = 0.1
CONFIDENCE_VALUE = 0.9
START_ANGLE = -90.0
END_ANGLE = 90.0

# pylint: disable=redefined-outer-name, duplicate-code


@pytest.fixture()
def vfh_default() -> vfh.VectorFieldHistogram:
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
def sample_lidar_oscillation() -> lidar_oscillation.LidarOscillation:
    """
    Creates a sample LidarOscillation instance with predefined readings.
    """
    readings = [
        lidar_oscillation.LidarDetection(1.0, -45.0),
        lidar_oscillation.LidarDetection(2.0, 0.0),
        lidar_oscillation.LidarDetection(1.5, 45.0),
    ]
    return lidar_oscillation.LidarOscillation(readings)


class TestVectorFieldHistogram:
    """
    Tests for the VectorFieldHistogram.run() method.
    """

    def test_initialization_with_default_parameters(
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

    def test_run_with_valid_lidar_oscillation(
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

    def test_run_with_none_oscillation(self, vfh_default: vfh.VectorFieldHistogram) -> None:
        """
        Test VectorFieldHistogram.run() method with None input.
        """
        result, polar_density = vfh_default.run(None)
        assert not result
        assert polar_density is None
