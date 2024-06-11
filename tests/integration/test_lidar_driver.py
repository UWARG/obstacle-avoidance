"""
Lightware SF45/B Testing (using serial connection).
"""

import time

from modules.detection import lidar_driver


PORT_NAME = "/dev/tty.usbmodem38S45_158681"
BAUDRATE = 921600
TIMEOUT = 0.1


def output_scan(lidar: lidar_driver.LidarDriver, duration: float) -> None:
    """
    Output lidar scans for number of reps.
    """
    time.sleep(2)

    t_end = time.time() + duration
    while time.time() < t_end:
        distance, angle = lidar.wait_for_reading(lidar.serial_port)

        if distance == -1:
            continue

        print(f"{distance} m {angle} deg")

    time.sleep(0.5)


def main() -> None:
    """
    Conduct tests of lidar driver methods.
    """
    lidar = lidar_driver.LidarDriver(PORT_NAME, BAUDRATE, TIMEOUT)

    print("Configuring lidar ...")
    # default lidar configuration (max speed, max rotation, 500 Hz update rate)
    lidar.set_update_rate(lidar.serial_port, 5)
    lidar.set_default_distance_output(lidar.serial_port)
    lidar.set_low_angle(lidar.serial_port, lidar.MIN_LOW_ANGLE)
    lidar.set_high_angle(lidar.serial_port, lidar.MAX_HIGH_ANGLE)
    lidar.set_speed(lidar.serial_port, lidar.MAX_SPEED)

    # start streaming
    lidar.set_default_distance_output(lidar.serial_port)
    lidar.set_distance_stream_enable(lidar.serial_port, True)

    print("Testing output:")
    output_scan(lidar, 3)

    print("Starting update rate test:")

    print("Increasing update rate to max (5000 Hz)")
    lidar.set_update_rate(lidar.serial_port, lidar.MAX_UPDATE_RATE)
    output_scan(lidar, 3)

    print("Decreasing update rate to min (50 Hz)")
    lidar.set_update_rate(lidar.serial_port, lidar.MIN_UPDATE_RATE)
    output_scan(lidar, 3)

    print("Starting rotation test:")

    print("Setting minimum rotation")
    lidar.set_low_angle(lidar.serial_port, lidar.MAX_LOW_ANGLE)
    lidar.set_high_angle(lidar.serial_port, lidar.MIN_HIGH_ANGLE)
    output_scan(lidar, 3)

    print("Setting maximum rotation")
    lidar.set_low_angle(lidar.serial_port, lidar.MIN_LOW_ANGLE)
    lidar.set_high_angle(lidar.serial_port, lidar.MAX_HIGH_ANGLE)
    output_scan(lidar, 3)

    print("Starting rotation speed test:")

    print("Setting minimum rotation speed")
    lidar.set_speed(lidar.serial_port, lidar.MIN_SPEED)
    output_scan(lidar, 3)

    print("Setting maximum rotation speed")
    lidar.set_speed(lidar.serial_port, lidar.MAX_SPEED)
    output_scan(lidar, 3)

    print("Done")


if __name__ == "__main__":
    main()
