"""
Module to parse LiDAR data, detect oscillations, and return oscillation objects.
"""

from modules import lidar_oscillation
from modules import lidar_detection


class LidarParser:
    """
    Class to handle parsing of LiDAR data stream and detecting complete oscillations.
    """

    __create_key = object()

    @classmethod
    def create(cls) -> "tuple[bool, LidarParser | None]":
        """
        Create a LidarParser object for processing live LiDAR data.
        """
        return True, LidarParser(cls.__create_key)

    def __init__(self, class_private_create_key: object) -> None:
        """
        Private constructor for LidarParser. Use create() method.
        """
        assert class_private_create_key is LidarParser.__create_key, "Use the create() method"

        self.lidar_readings = []
        self.current_oscillation = None

        self.last_angle = None
        self.direction = None
        self.__run = False

    def process_reading(
        self, lidar_detection: lidar_detection.LidarDetection
    ) -> "tuple[bool, lidar_oscillation.LidarOscillation | None]":
        """
        Process a single LiDAR detection object and check for oscillation completion.
        """

        if lidar_detection is None:
            print(f"Invalid reading: {lidar_detection}")
            return False, None

        self.lidar_readings.append(lidar_detection)

        # Oscillation detection logic
        current_angle = lidar_detection.angle
        if self.last_angle is None:
            self.last_angle = current_angle
            return False, None

        if current_angle > self.last_angle and self.direction == "down":
            self.complete_oscillation()
            self.direction = "up"
            return True, self.current_oscillation
        elif current_angle < self.last_angle and self.direction == "up":
            self.complete_oscillation()
            self.direction = "down"
            return True, self.current_oscillation
        elif self.direction is None:
            self.direction = "up" if current_angle > self.last_angle else "down"

        self.last_angle = current_angle
        return False, None

    def complete_oscillation(self) -> None:
        """
        Mark the current oscillation as complete and prepare for the next one.
        """
        result, oscillation = lidar_oscillation.LidarOscillation.create(self.lidar_readings)
        if result:
            self.current_oscillation = oscillation
            print(oscillation)
        else:
            print("Error creating LidarOscillation")

        self.lidar_readings = []

    def run(self, lidar_detection: lidar_detection.LidarDetection) -> None:
        """
        Process a single LidarDetection.
        """
        if not self.__run:
            self.__run = True

        return self.process_reading(lidar_detection)
