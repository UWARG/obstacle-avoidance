import numpy as np

from modules import lidar_oscillation
from modules import polar_obstacle_density


class VectorFieldHistogram:
    """
    Class to generate PolarObstacleDensity from a LidarOscillation using the Vector Field Histogram (VFH) method.
    """

    def __init__(
        self,
        sector_width: float,
        max_vector_magnitude: float,
        linear_decay_rate: float,
        confidence_value: float,
        start_angle: float,
        end_angle: float,
    ) -> None:
        """
        Initialize the VectorFieldHistogram with the following parameters:

        - sector_width (degrees): The angular size of each sector.
        - max_vector_magnitude (unitless): The maximum magnitude applied to obstacle vectors in
        SectorObstacleDensities, representing the highest obstacle density for a sector.
        - linear_decay_rate (unitless): The rate at which the magnitude of SectorObstacleDensity
        reduces with increasing distance.
        - confidence_value (unitless): The certainty value applied to each detection, to adjust for LiDAR error.
        """

        if sector_width <= 0:
            sector_width = 2
        if not (0 <= max_vector_magnitude <= 1):
            max_vector_magnitude = 1
        if not (0 <= linear_decay_rate <= 1):
            linear_decay_rate = 0.1
        if not (0 <= confidence_value <= 1):
            confidence_value = 0.9
        if start_angle >= end_angle:
            start_angle = -90
            end_angle = 90

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

        object_densities = np.zeros(self.num_sectors)

        for reading in oscillation.readings:
            angle = reading.angle

            if angle < self.start_angle or angle > self.end_angle:
                continue

            # Convert angle to sector index
            sector_index = int((angle - self.start_angle) / self.sector_width)
            sector_index = min(sector_index, self.num_sectors - 1)

            distance_factor = self.max_vector_magnitude - self.linear_decay_rate * reading.distance
            obstacle_magnitude = (self.confidence_value**2) * distance_factor

            # Accumulate obstacle densities
            object_densities[sector_index] += max(obstacle_magnitude, 0)

        sector_densities = []
        for i, density in enumerate(object_densities):
            angle_start = self.start_angle + i * self.sector_width
            angle_end = angle_start + self.sector_width
            result, sector_density = polar_obstacle_density.SectorObstacleDensity.create(
                angle_start, angle_end, density
            )
            if result:
                sector_densities.append(sector_density)

        result, polar_obstacle_density_object = polar_obstacle_density.PolarObstacleDensity.create(
            sector_densities
        )
        return result, polar_obstacle_density_object
