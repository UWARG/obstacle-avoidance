"""
Gets detections and outputs oscillations when complete.
"""

from . import lidar_parser
from modules import lidar_detection
from modules import lidar_oscillation
from worker import queue_wrapper
from worker import worker_controller


def lidar_oscillation_worker(
    detection_in_queue: queue_wrapper.QueueWrapper,
    oscillation_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Feeding LidarParser continuously with a stream of LidarDetection
    """

    result, parser = lidar_parser.LidarParser()
    if not result:
        print("Failed to initialise LidarParser.")
        return

    while not controller.is_exit_requested():
        controller.check_pause()

        lidar_reading: lidar_detection.LidarDetection = detection_in_queue.queue.get()
        if lidar_reading is None:  # Do we want this line?
            break

        result, oscillation = parser.run(lidar_reading)
        if not result:
            continue

        print(f"Oscillation sent to VFH")
        oscillation_out_queue.queue.put(oscillation)
