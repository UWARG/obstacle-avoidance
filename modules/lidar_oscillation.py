from lidar_detection import LidarDetection

"""
Representing a LiDAR Oscillation
"""

class LidarOscillation:
    """
    Class to represent a collection of LiDAR readings that make up an oscillation.
    """

    __create_key = object()

    @classmethod
    def create(cls, readings: list[LidarDetection]) -> "tuple[bool, LidarOscillation | None]":
        """
        Create a new LidarOscillation object from a list of LidarReading objects.
        """
        # Ensuring the list does not contain None values
        if any(reading is None for reading in readings):
            return False, None

        return True, LidarOscillation(cls.__create_key, readings)

    def __init__(self, class_private_create_key: object, readings: list[LidarDetection]) -> None:
        """
        Private constructor, use create() method to instantiate.
        """
        assert class_private_create_key is LidarOscillation.__create_key, "Use the create() method"

        self.readings = readings
        valid_angles = [reading.angle for reading in readings if reading is not None]

        if valid_angles:
            self.min_angle = min(valid_angles)
            self.max_angle = max(valid_angles)
        else:
            self.min_angle = None
            self.max_angle = None
         
    def __str__(self) -> str:
        """
        Return a string representation of the LiDAR oscillation data.
        """
        reading_strs = []
        for reading in self.readings:
            reading_strs.append(str(reading))
        formatted_readings = ", ".join(reading_strs)
        return f"LidarOscillation: {len(self.readings)} readings, Min angle: {self.min_angle}, Max angle: {self.max_angle}, Readings: {formatted_readings}."

