"""
Detects objects using lidar and combines with current odometry
"""

from .. import drone_odometry_local
from .. import detection_and_odometry
from . import lidar_driver


class Detection:
    """
    Configures and receives lidar data
    """

    def __init__(self, serial_port_name: str, serial_port_baudrate: int, timeout: float) -> None:
        """
        serial_port_name: port that the lidar is connected to
        serial_port_baudrate: baudrate of the lidar port
        timeout: timeout for connecting to serial port
        """
        self.lidar = lidar_driver.LidarDriver(serial_port_name, serial_port_baudrate, timeout)

        # configure lidar
        self.lidar.set_update_rate(self.lidar.serial_port, 1)
        self.lidar.set_default_distance_output(self.lidar.serial_port)
        self.lidar.set_low_angle(self.lidar.serial_port, -170)
        self.lidar.set_high_angle(self.lidar.serial_port, 5)
        self.lidar.set_speed(self.lidar.serial_port, 5)

    def run(
        self, odometry: drone_odometry_local.DroneOdometryLocal
    ) -> "tuple[bool, detection_and_odometry.DetectionAndOdometry | None]":
        """
        Returns a possible DetectionAndOdometry.
        """

        if odometry is None:
            return False, None

        distance, angle = self.lidar.wait_for_reading(self.sensor_port)

        result, lidar_detection = detection_and_odometry.LidarDetection.create(distance, angle)
        if not result:
            return False, None

        return detection_and_odometry.DetectionAndOdometry.create(lidar_detection, odometry)
