"""
Detection worker integration test.
"""

import multiprocessing as mp
import time

from worker import worker_controller
from worker import queue_wrapper

from modules import drone_odometry_local
from modules import detection_and_odometry
from modules.detection import detection_worker
from ..common.mavlink.modules import drone_odometry

# Constants
WORK_COUNT = 3
DELAY_FOR_SIMULATED_WORKER = 1  # seconds

SERIAL_PORT_NAME = "/dev/tty.usbmodem38S45_158681"
SERIAL_PORT_BAUDRATE = 921600
PORT_TIMEOUT = 0.1  # seconds


def simulate_previous_worker(in_queue):
    """
    Place example odometry into the queue.
    """
    result, position = drone_odometry_local.DronePositionLocal.create(5.0, 5.0, -5.0)
    assert result
    assert position is not None

    result, orientation = drone_odometry.DroneOrientation.create(0.0, 0.0, 0.0)
    assert result
    assert orientation is not None

    result, odometry = drone_odometry_local.DroneOdometryLocal.create(position, orientation)
    assert result
    assert odometry is not None

    in_queue.queue.put(odometry)


def main():
    """
    Main function.
    """
    controller = worker_controller.WorkerController()
    mp_manager = mp.Manager()

    odometry_in_queue = queue_wrapper.QueueWrapper(mp_manager)
    detection_out_queue = queue_wrapper.QueueWrapper(mp_manager)

    worker = mp.Process(
        target=detection_worker.detection_worker,
        args=(
            SERIAL_PORT_NAME,
            SERIAL_PORT_BAUDRATE,
            PORT_TIMEOUT,
            odometry_in_queue,
            detection_out_queue,
            controller,
        ),
    )

    worker.start()

    for _ in range(0, WORK_COUNT):
        simulate_previous_worker(odometry_in_queue)

    time.sleep(DELAY_FOR_SIMULATED_WORKER)

    for _ in range(0, WORK_COUNT):
        input_data: detection_and_odometry.DetectionAndOdometry = detection_out_queue.get_nowait()
        print(input_data)
        assert input_data is not None

    assert detection_out_queue.queue.empty()

    controller.request_exit()
    odometry_in_queue.fill_and_drain_queue()
    worker.join()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Error: Status Code: {result_main}")

    print("Done.")
