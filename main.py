"""
Main
"""

import multiprocessing as mp
import queue

from .worker import worker_controller
from .worker import queue_wrapper

from ..modules import drone_odometry_local
from .modules.flight_interface import flight_interface_worker


def main() -> int:
    """
    Main function
    """

    QUEUE_MAX_SIZE = 10
    FLIGHT_INTERFACE_ADDRESS = "tcp:127.0.0.1:14550"
    FLIGHT_INTERFACE_TIMEOUT = 10
    FLIGHT_INTERFACE_WORKER_PERIOD = 0.1

    controller = worker_controller.WorkerController()
    manager = mp.Manager()

    flight_interface_to_main_queue = queue_wrapper.QueueWrapper(manager, QUEUE_MAX_SIZE)

    args = (
        FLIGHT_INTERFACE_ADDRESS,
        FLIGHT_INTERFACE_TIMEOUT,
        FLIGHT_INTERFACE_WORKER_PERIOD,
        flight_interface_to_main_queue,
        controller,
    )

    worker = mp.Process(target=flight_interface_worker, args=args)

    worker.start()

    while True:
        try:
            input_data: drone_odometry_local.DroneOdometryLocal = (
                flight_interface_to_main_queue.queue.get()
            )
            assert (
                str(type(input_data))
                == "<class 'modules.drone_odometry_local\
                    .DroneOdometryLocal'>"
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
    main_result = main()
    if main_result < 0:
        print("error: " + main_result)
    print(main_result)

    print("Done.")
