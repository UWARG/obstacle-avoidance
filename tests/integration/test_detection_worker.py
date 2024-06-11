"""
Detection worker integration test.
"""

import multiprocessing as mp
import queue

from worker import worker_controller
from worker import queue_wrapper

from modules import lidar_detection
from modules.detection import detection_worker


# Constants
QUEUE_MAX_SIZE = 10

SERIAL_PORT_NAME = "/dev/tty.usbmodem38S45_158681"
SERIAL_PORT_BAUDRATE = 921600
PORT_TIMEOUT = 0.1  # seconds
UPDATE_RATE = 5
HIGH_ANGLE = 170
LOW_ANGLE = -170
ROTATE_SPEED = 5


def main() -> int:
    """
    Main function.
    """
    controller = worker_controller.WorkerController()
    mp_manager = mp.Manager()

    detection_out_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)

    worker = mp.Process(
        target=detection_worker.detection_worker,
        args=(
            SERIAL_PORT_NAME,
            SERIAL_PORT_BAUDRATE,
            PORT_TIMEOUT,
            UPDATE_RATE,
            HIGH_ANGLE,
            LOW_ANGLE,
            ROTATE_SPEED,
            detection_out_queue,
            controller,
        ),
    )

    worker.start()

    while True:
        try:
            input_data: detection_and_odometry.DetectionAndOdometry = (
                detection_out_queue.queue.get_nowait()
            )
            assert str(type(input_data)) == "<class 'modules.lidar_detection.LidarDetection'>"

            assert input_data is not None

            print(input_data)

        except queue.Empty:
            break

    controller.request_exit()
    worker.join()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Error: Status Code: {result_main}")

    print("Done.")
