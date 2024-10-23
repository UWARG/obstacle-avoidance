import numpy as np

from modules import lidar_oscillation
from modules import polar_obstacle_density


class VectorFieldHistogram:
    """
    Class to generate PolarObstacleDensity from a LidarOscillation using the Vector Field Histogram (VFH) method.
    """

    def __init__(
        self,
        sector_width: float = 2.0,
        max_vector_magnitude: float = 1.0,
        linear_decay_rate: float = 0.1,
        confidence_value: float = 0.9,
        start_angle: float = -90.0,
        end_angle: float = 90.0,
    ) -> None:
        """
        Initialize the VectorFieldHistogram with the following parameters:

        - sector_width: The angular size of each sector.
        - max_vector_magnitude: The maximum magnitude of obstacle vectors.
        - linear_decay_rate: The rate at which vector magnitudes decay with distance.
        - confidence_value: The certainty value applied to each detection.
        - density_threshold: The threshold for obstacle density filtering (used later).
        """

        if sector_width <= 0:
            raise ValueError("Sector width must be greater than 0.")
        if not (0 <= max_vector_magnitude <= 1):
            raise ValueError("max_vector_magnitude must be between 0 and 1.")
        if not (0 <= linear_decay_rate <= 1):
            raise ValueError("linear_decay_rate must be between 0 and 1.")
        if not (0 <= confidence_value <= 1):
            raise ValueError("confidence_value must be between 0 and 1.")
        if start_angle >= end_angle:
            raise ValueError("start_angle must be less than end_angle.")

        self.sector_width = sector_width
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.num_sectors = int((end_angle - start_angle) / self.sector_width)

        self.max_vector_magnitude = max_vector_magnitude
        self.linear_decay_rate = linear_decay_rate
        self.confidence_value = confidence_value

    def run(
        self, oscillation: lidar_oscillation.LidarOscillation
    ) -> "tuple[bool, polar_obstacle_density.PolarObstacleDensity | None]":
        """
        Generate a PolarObstacleDensity from the LidarOscillation.
        """
        if oscillation is None:
            return False, None

        object_densities = np.zeros(self.num_sectors)

        for reading in oscillation.readings:
            angle = reading.angle

            if angle < self.start_angle or angle > self.end_angle:
                continue

            # Convert angle to sector index
            sector_index = int((angle - self.start_angle) / self.sector_width)
            sector_index = min(sector_index, self.num_sectors - 1)

            distance_factor = self.max_vector_magnitude - self.linear_decay_rate * reading.distance
            m_ij = (self.confidence_value**2) * distance_factor

            # Accumulate obstacle densities
            object_densities[sector_index] += max(m_ij, 0)

        sector_densities = []
        for i, density in enumerate(object_densities):
            angle_start = self.start_angle + i * self.sector_width
            angle_end = angle_start + self.sector_width
            result, sector_density = polar_obstacle_density.SectorObstacleDensity.create(
                angle_start, angle_end, density
            )
            if result:
                sector_densities.append(sector_density)

        result, polar_obstacle_density = polar_obstacle_density.PolarObstacleDensity.create(
            sector_densities
        )
        return result, polar_obstacle_density
