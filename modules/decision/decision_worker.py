"""
Gets detections and odometry and outputs a decision.
"""

from worker import queue_wrapper
from worker import worker_controller
from modules import detections_and_odometry
from . import decision


def decision_worker(
    merged_in_queue: queue_wrapper.QueueWrapper,
    command_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process

    merged_in_queue, command_out_queue are data queues.
    controller is how the main process communicates to this worker process.
    """

    decider = decision.Decision("HALTED")

    while not controller.is_exit_requested():
        controller.check_pause()

        merged_data: detections_and_odometry.DetectionsAndOdometry = merged_in_queue.queue.get()
        if merged_data is None:
            continue

        decider.detection_and_odometries.append(merged_data)

        result, value = decider.run(decider.detection_and_odometries)
        if not result:
            continue

        command_out_queue.queue.put(value)
