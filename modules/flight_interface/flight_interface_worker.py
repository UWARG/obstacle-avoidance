"""
Fetches local drone odometry.
"""

import time
import queue

from modules import decision_command
from worker import queue_wrapper
from worker import worker_controller
from . import flight_interface


def flight_interface_worker(
    address: str,
    timeout: float,
    period: float,
    command_in_queue: queue_wrapper.QueueWrapper,
    odometry_out_queue: queue_wrapper.QueueWrapper,
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
        if result:
            odometry_out_queue.queue.put(value)

        try:
            command: decision_command.DecisionCommand = command_in_queue.queue.get_nowait()
            if command is None:
                break
        except queue.Empty:
            continue

        result = interface.run_decision_handler(command)
        if result:
            print("Command received and sent to drone.")
