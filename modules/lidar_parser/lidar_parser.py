"""
Module to parse LiDAR data, detect oscillations, and return oscillation objects.
"""

import enum

from modules import lidar_detection
from modules import lidar_oscillation


class Direction(enum.Enum):
    """
    Enum for LiDAR scan direction.
    """

    UP = 1
    DOWN = 2
    NONE = 3


class LidarParser:
    """
    Class to handle parsing of LiDAR data stream and detecting complete oscillations.
    """

    def __init__(self) -> None:
        """
        Private constructor for LidarParser. Use create() method.
        """

        self.lidar_readings = []
        self.current_oscillation = None

        self.last_angle = None
        self.direction = Direction.NONE

    def run(
        self, detection: lidar_detection.LidarDetection
    ) -> "tuple[bool, lidar_oscillation.LidarOscillation | None]":
        """
        Process a single LidarDetection and return the oscillation if complete.
        """
        current_angle = detection.angle

        if self.last_angle is None:
            self.last_angle = current_angle
            self.lidar_readings.append(detection)
            return False, None

        # Detect oscillation on angle change with correct direction reset
        if current_angle > self.last_angle and self.direction == Direction.DOWN:
            result, oscillation = lidar_oscillation.LidarOscillation.create(self.lidar_readings)
            self.direction = Direction.UP
            self.lidar_readings = [detection]
            self.last_angle = current_angle
            return result, oscillation

        if current_angle < self.last_angle and self.direction == Direction.UP:
            result, oscillation = lidar_oscillation.LidarOscillation.create(self.lidar_readings)
            self.direction = Direction.DOWN
            self.lidar_readings = [detection]
            self.last_angle = current_angle
            return result, oscillation

        if self.direction is Direction.NONE:
            self.direction = Direction.UP if current_angle > self.last_angle else Direction.DOWN

        self.lidar_readings.append(detection)
        self.last_angle = current_angle
        return False, None
