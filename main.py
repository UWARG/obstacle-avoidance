"""
Main
"""

import multiprocessing as mp
import queue

from worker import worker_controller
from worker import queue_wrapper

from modules import drone_odometry_local
from modules import lidar_detection

from modules.flight_interface import flight_interface_worker
from modules.detection import detection_worker


def main() -> int:
    """
    Main function
    """

    # Local constants
    # pylint: disable=invalid-name
    QUEUE_MAX_SIZE = 10

    FLIGHT_INTERFACE_ADDRESS = "tcp:127.0.0.1:14550"
    FLIGHT_INTERFACE_TIMEOUT = 10.0
    FLIGHT_INTERFACE_WORKER_PERIOD = 0.1

    SERIAL_PORT_NAME = "/dev/tty.usbmodem38S45_158681"
    SERIAL_PORT_BAUDRATE = 921600
    PORT_TIMEOUT = 0.1  # seconds
    UPDATE_RATE = 5
    HIGH_ANGLE = 170
    LOW_ANGLE = -170
    ROTATE_SPEED = 5
    # pylint: enable=invalid-name

    controller = worker_controller.WorkerController()
    manager = mp.Manager()

    flight_interface_to_main_queue = queue_wrapper.QueueWrapper(manager, QUEUE_MAX_SIZE)

    detection_to_main_queue = queue_wrapper.QueueWrapper(manager, QUEUE_MAX_SIZE)

    flight_interface_process = mp.Process(
        target=flight_interface_worker.flight_interface_worker,
        args=(
            FLIGHT_INTERFACE_ADDRESS,
            FLIGHT_INTERFACE_TIMEOUT,
            FLIGHT_INTERFACE_WORKER_PERIOD,
            flight_interface_to_main_queue,
            controller,
        ),
    )

    detection_process = mp.Process(
        target=detection_worker.detection_worker,
        args=(
            SERIAL_PORT_NAME,
            SERIAL_PORT_BAUDRATE,
            PORT_TIMEOUT,
            UPDATE_RATE,
            HIGH_ANGLE,
            LOW_ANGLE,
            ROTATE_SPEED,
            detection_to_main_queue,
            controller,
        ),
    )

    flight_interface_process.start()
    detection_process.start()

    while True:
        try:
            flight_interface_data: drone_odometry_local.DroneOdometryLocal = (
                flight_interface_to_main_queue.queue.get_nowait()
            )

            if flight_interface_data is not None:
                print(flight_interface_data)

            detection_data: lidar_detection.LidarDetection = (
                detection_to_main_queue.queue.get_nowait()
            )

            if detection_data is not None:
                print(detection_data)

        except KeyboardInterrupt:
            controller.request_exit()
            break

    flight_interface_to_main_queue.fill_and_drain_queue()
    detection_to_main_queue.fill_and_drain_queue()

    flight_interface_process.join()
    detection_process.join()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Error: Status Code: {result_main} ")
    print(result_main)

    print("Done.")
