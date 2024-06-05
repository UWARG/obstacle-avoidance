"""
Fetches local drone odometry.
"""

import time

from . import flight_interface
from worker import queue_wrapper
from worker import worker_controller


def flight_interface_worker(
    address: str,
    timeout: float,
    period: float,
    output_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    address, timeout is initial setting.
    period is minimum period between loops.
    output_queue is the data queue.
    controller is how the main process communicates to this worker process.
    """
    result, interface = flight_interface.FlightInterface.create(address, timeout)
    if not result:
        return

    while not controller.is_exit_requested():
        controller.check_pause()

        time.sleep(period)

        result, value = interface.run()
        if not result:
            continue

        output_queue.queue.put(value)
