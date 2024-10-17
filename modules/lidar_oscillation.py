"""
Representing a LiDAR Oscillation
"""

class LidarReading:
    """
    Class to represent a LiDAR oscillation with distance and angle readings.
    """

    __create_key = object()

    @classmethod
    def create(cls, distance: float, angle: float) -> "tuple[bool, LidarReading | None]":
        """
        Create a new LidarReading object.
        """
        # Ensure valid input data 
        if distance < 0 or angle < -170 or angle > 170:
            return False, None

        return True, LidarReading(cls.__create_key, distance, angle)

    def __init__(self, class_private_create_key: object, distance: float, angle: float) -> None:
        """
        Private constructor, use create() method for instantiation
        """
        assert class_private_create_key is LidarReading.__create_key, "Use the create() method"

        # Store the distance and angle for this oscillation
        self.distance = distance
        self.angle = angle

    def __str__(self) -> str:
        """
        Return a string representation of the LiDAR oscillation data.
        """
        return f"LidarReading: {self.distance} m at {self.angle} degrees"

class LidarOscillation:
    """
    Class to represent a collection of LiDAR readings that make up an oscillation.
    """

    __create_key = object()

    @classmethod
    def create(cls, readings: list[LidarReading]) -> "tuple[bool, LidarOscillation | None]":
        """
        Create a new LidarOscillation object from a list of LidarReading objects.
        """
        # Ensuring the list only contains LidarReading instances
        if not all(isinstance(reading, LidarReading) for reading in readings):
            return False, None

        return True, LidarOscillation(cls.__create_key, readings)

    def __init__(self, class_private_create_key: object, readings: list) -> None:
        """
        Private constructor, use create() method to instantiate.
        """
        assert class_private_create_key is LidarOscillation.__create_key, "Use the create() method"

        # Store the list of LidarReading objects
        self.readings = readings

    def __str__(self) -> str:
        """
        Return a string representation of the LiDAR oscillation data.
        """
        reading_strs = [str(reading) for reading in self.readings]  # Create a list of reading strings
        return f"LidarOscillation with {len(self.readings)} readings:\n" + "\n".join(reading_strs)


