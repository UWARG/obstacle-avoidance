"""
Fetches local drone odometry.
"""

import time
import queue

from modules import decision_command
from modules import odometry_and_waypoint
from worker import queue_wrapper
from worker import worker_controller
from . import flight_interface


def flight_interface_worker(
    address: str,
    timeout: float,
    first_waypoint_distance_tolerance: float,
    period: float,
    command_in_queue: queue_wrapper.QueueWrapper,
    odometry_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    address, timeout is initial setting.
    period is minimum period between loops.
    command_in_queue, odometry_out_queue are the data queues.
    controller is how the main process communicates to this worker process.
    """

    result, interface = flight_interface.FlightInterface.create(
        address, timeout, first_waypoint_distance_tolerance
    )
    if not result:
        return

    while not controller.is_exit_requested():
        controller.check_pause()

        time.sleep(period)

        result, value = interface.run()
        if result:
            if value.flight_mode == odometry_and_waypoint.FlightMode.MANUAL:
                print("Obstacle avoidance killed. Check flight mode.")
                controller.request_exit()
                break
            odometry_out_queue.queue.put(value)

        try:
            command: decision_command.DecisionCommand = command_in_queue.queue.get_nowait()
            if command is None:
                break
        except queue.Empty:
            continue

        result = interface.run_decision_handler(command)
