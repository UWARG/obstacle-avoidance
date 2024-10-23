"""
Representing a LiDAR Oscillation
"""

from . import lidar_detection


class LidarOscillation:
    """
    Class to represent a collection of LiDAR readings that make up an oscillation.
    """

    __create_key = object()

    @classmethod
    def create(
        cls, readings: list[lidar_detection.LidarDetection]
    ) -> "tuple[bool, LidarOscillation | None]":
        """
        Create a new LidarOscillation object from a list of LidarReading objects.
        """
        if not readings:
            return False, None
        return True, LidarOscillation(cls.__create_key, readings)

    def __init__(
        self, class_private_create_key: object, readings: list[lidar_detection.LidarDetection]
    ) -> None:
        """
        Private constructor, use create() method to instantiate.
        """
        assert class_private_create_key is LidarOscillation.__create_key, "Use the create() method"

        self.readings = readings
        angles = [reading.angle for reading in readings]
        self.min_angle = min(angles)
        self.max_angle = max(angles)

    def __str__(self) -> str:
        """
        Return a string representation of the LiDAR oscillation data.
        """
        reading_strs = []
        for reading in self.readings:
            reading_strs.append(str(reading))
        formatted_readings = ", ".join(reading_strs)
        return f"LidarOscillation: {len(self.readings)} readings, Min angle: {self.min_angle}, Max angle: {self.max_angle}, Readings: {formatted_readings}."
