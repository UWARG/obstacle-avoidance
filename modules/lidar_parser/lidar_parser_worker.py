"""
Gets detections and outputs oscillations when complete.
"""

from modules import lidar_detection
from modules import lidar_oscillation
from modules import decision
from worker import queue_wrapper
from worker import worker_controller
from . import lidar_parser


def lidar_oscillation_worker(
    detection_in_queue: queue_wrapper.QueueWrapper,
    oscillation_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Feeding LidarParser continuously with a stream of LidarDetection
    """

    result, parser = lidar_parser.LidarParser.create()
    if not result:
        print("Failed to create LidarParser.")
        return

    while not controller.is_exit_requested():
        controller.check_pause()

        lidar_data: lidar_detection.LidarDetection = detection_in_queue.queue.get()
        if lidar_data is None:
            break

        parser.run(lidar_data)

        result, oscillation = parser.run(detection_in_queue)
        if not result:
            continue

        print(f"Oscillation sent to VFH")
        oscillation_out_queue.queue.put(oscillation)
