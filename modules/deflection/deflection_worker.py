"""
Gets obstacles and odometry and outputs a decision.
"""

from modules import obstacles_and_odometry
from worker import queue_wrapper
from worker import worker_controller
from . import deflection


def deflection_worker(
    cluster_in_queue: queue_wrapper.QueueWrapper,
    command_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    cluster_in_queue, command_out_queue are data queues.
    controller is how the main process communicates to this worker process.
    """

    deflecter = deflection.Deflection()

    while not controller.is_exit_requested():
        controller.check_pause()

        cluster_data: obstacles_and_odometry.ObstaclesAndOdometry = cluster_in_queue.queue.get()
        if cluster_data is None:
            break

        result, value = deflecter.run() # run with cluster_data
        if not result:
            continue

        command_out_queue.queue.put(value)
