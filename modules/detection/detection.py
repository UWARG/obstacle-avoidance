"""
Detects objects using lidar and combines with current odometry
"""

from .. import lidar_detection
from . import lidar_driver


class Detection:
    """
    Configures and receives lidar data
    """

    def __init__(
        self,
        serial_port_name: str,
        serial_port_baudrate: int,
        timeout: float,
        update_rate: int,
        low_angle: float,
        high_angle: float,
        rotate_speed: int,
    ) -> None:
        """
        serial_port_name: port that the lidar is connected to
        serial_port_baudrate: baudrate of the lidar port
        timeout: timeout for connecting to serial port
        update_rate: frequency the lidar reads points (must be integer between 1 and 12 inclusive)
        low_angle: lidar low angle in degrees (must be between -170 and -5 inclusive)
        high_angle: lidar high angle in degrees (must be between 5 and 170 inclusive)
        rotate_speed: lidar rotational speed (must be integer between 5 and 2000 inclusive where 5 is the fastest)
        """
        self.update_rate = update_rate
        self.low_angle = low_angle
        self.high_angle = high_angle
        self.rotate_speed = rotate_speed

        self.lidar = lidar_driver.LidarDriver(serial_port_name, serial_port_baudrate, timeout)

        # configure lidar
        self.lidar.set_update_rate(self.lidar.serial_port, self.update_rate)
        self.lidar.set_default_distance_output(self.lidar.serial_port)
        self.lidar.set_low_angle(self.lidar.serial_port, self.low_angle)
        self.lidar.set_high_angle(self.lidar.serial_port, self.high_angle)
        self.lidar.set_speed(self.lidar.serial_port, self.rotate_speed)

        # start streaming
        self.lidar.set_default_distance_output(self.lidar.serial_port)
        self.lidar.set_distance_stream_enable(self.lidar.serial_port, True)

    def run(self) -> "tuple[bool, lidar_detection.LidarDetection | None]":
        """
        Returns a possible LidarDetection.
        """

        distance, angle = self.lidar.wait_for_reading(self.lidar.serial_port)

        return lidar_detection.LidarDetection.create(distance, angle)
