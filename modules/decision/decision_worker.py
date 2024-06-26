"""
Gets detections and odometry and outputs a decision.
"""

from worker import queue_wrapper
from worker import worker_controller
from modules import detections_and_odometry
from . import decision


def decision_worker(
    initial_drone_state: decision.Decision.DroneState,
    object_proximity_limit: float,
    max_history: int,
    merged_in_queue: queue_wrapper.QueueWrapper,
    command_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process

    object_proximity_limit is the minimum distance the drone will maintain from an object (in metres).
    merged_in_queue, command_out_queue are data queues.
    controller is how the main process communicates to this worker process.
    """

    decider = decision.Decision(initial_drone_state, object_proximity_limit, max_history)

    while not controller.is_exit_requested():
        controller.check_pause()

        merged_data: detections_and_odometry.DetectionsAndOdometry = merged_in_queue.queue.get()
        if merged_data is None:
            break

        result, value = decider.run(merged_data)
        if not result:
            continue

        command_out_queue.queue.put(value)
