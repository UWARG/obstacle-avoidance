"""
Lightware SF45/B Testing (using serial connection).
"""

import time

from modules.detection import lidar_driver


PORT_NAME = "/dev/tty.usbmodem38S45_158681"
BAUDRATE = 921600
TIMEOUT = 0.1


def output_scan(lidar, reps, min_angle=-170, max_min_angle=-5, min_max_angle=5, max_angle=170):
    """
    Output lidar scans for number of reps.
    """
    for _ in (0, reps):
        distance, angle = lidar.wait_for_reading(lidar.serial_port)

        assert distance != -1
        assert angle >= min_angle
        assert angle <= max_angle
        assert angle <= max_min_angle or angle >= min_max_angle

        print(f"{distance} m {angle} deg")


def main():
    """
    Conduct tests of lidar driver methods.
    """
    lidar = lidar_driver.LidarDriver(PORT_NAME, BAUDRATE, TIMEOUT)

    lidar.set_default_distance_output(lidar.serial_port)

    print("Testing output:")
    output_scan(lidar, 50)

    time.sleep(2)

    print("Starting update rate test:")

    print("Increasing update rate to max (5000 Hz)")
    lidar.set_update_rate(lidar.serial_port, 12)
    output_scan(lidar, 5000)

    print("Decreasing update rate to min (50 Hz)")
    lidar.set_update_rate(lidar.serial_port, 1)
    output_scan(lidar, 200)

    time.sleep(2)

    print("Starting rotation test:")

    print("Setting minimum rotation")
    lidar.set_low_angle(lidar.serial_port, -5)
    lidar.set_high_angle(lidar.serial_port, 5)
    output_scan(lidar, 200, min_angle=-5, max_angle=5)

    print("Setting maximum rotation")
    lidar.set_low_angle(lidar.serial_port, -170)
    lidar.set_high_angle(lidar.serial_port, 170)
    output_scan(lidar, 200)

    time.sleep(2)

    print("Starting rotation speed test:")

    print("Setting minimum rotation speed")
    lidar.set_speed(lidar.serial_port, 2000)
    output_scan(lidar, 200)

    print("Setting maximum rotation speed")
    lidar.set_speed(lidar.serial_port, 5)
    output_scan(lidar, 200)

    print("Done")
