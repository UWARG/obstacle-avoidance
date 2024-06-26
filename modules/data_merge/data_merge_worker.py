"""
Merges local drone odometry with LiDAR detections
"""

import queue
import time

from worker import queue_wrapper
from worker import worker_controller

from modules import detections_and_odometry
from modules import drone_odometry_local
from modules import lidar_detection


def data_merge_worker(
    delay: float,
    detection_input_queue: queue_wrapper.QueueWrapper,
    odometry_input_queue: queue_wrapper.QueueWrapper,
    output_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.
    Expects lidar detections to be more frequent than odometry readings.

    detection_input_queue, odometry_input_queue, output_queue are data queues.
    controller is how the main process communicates to this worker process.
    """
    detections = []
    while not controller.is_exit_requested():
        controller.check_pause()

        try:
            detection: lidar_detection.LidarDetection = detection_input_queue.queue.get_nowait()
            detections.append(detection)
            print("Data merge: Received and added detection.")
        except queue.Empty:
            print("Data merge: No detection received.")
            time.sleep(delay)

        try:
            odometry: drone_odometry_local.DroneOdometryLocal = (
                odometry_input_queue.queue.get_nowait()
            )
            print("Data merge: Received odometry.")

        except queue.Empty:
            print("Data merge: No odometry received")
            continue

        result, merged = detections_and_odometry.DetectionsAndOdometry.create(detections, odometry)

        if not result:
            continue

        print(f"Data merge: Successfully merged detections and odometry: {merged}")
        detections = []
        output_queue.queue.put(merged)
