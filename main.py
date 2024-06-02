"""
Main
"""

import multiprocessing as mp
import queue

<<<<<<< HEAD
from worker import worker_controller
from worker import queue_wrapper

from modules import drone_odometry_local
from modules.flight_interface import flight_interface_worker
=======
from .worker import worker_controller
from .worker import queue_wrapper

from ..modules import drone_odometry_local
from .modules.flight_interface import flight_interface_worker
>>>>>>> 621fd6e (added main script)


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
    # pylint: enable=invalid-name

    controller = worker_controller.WorkerController()
    manager = mp.Manager()

    flight_interface_to_main_queue = queue_wrapper.QueueWrapper(manager, QUEUE_MAX_SIZE)

    worker = mp.Process(
        target=flight_interface_worker,
        args=(
            FLIGHT_INTERFACE_ADDRESS,
            FLIGHT_INTERFACE_TIMEOUT,
            FLIGHT_INTERFACE_WORKER_PERIOD,
            flight_interface_to_main_queue,
            controller,
        ),
    )

    worker.start()

    while True:
        try:
            input_data: drone_odometry_local.DroneOdometryLocal = (
                flight_interface_to_main_queue.queue.get_nowait()
            )
            assert (
                str(type(input_data)) == "<class 'modules.drone_odometry_local.DroneOdometryLocal'>"
            )
            assert input_data.local_position is not None
            assert input_data.drone_orientation is not None
        except queue.Empty:
            break

    controller.request_exit()
    flight_interface_to_main_queue.fill_and_drain_queue()
    worker.join()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Error: Status Code: {result_main} ")
    print(result_main)

    print("Done.")
