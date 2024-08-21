"""
Merges local drone odometry with LiDAR detections
"""

import enum
import queue
import time

from modules import detections_and_odometry
from modules import drone_odometry_local
from modules import lidar_detection
from modules import obstacle
from modules import obstacles_and_odometry

from worker import queue_wrapper
from worker import worker_controller


class MergeDataType(enum.Enum):
    """
    Types of data to merge with odometry.
    """

    OBSTACLES = 0
    DETECTIONS = 0


def data_merge_worker(
    delay: float,
    merge_data_type: str,
    detection_input_queue: queue_wrapper.QueueWrapper,
    obstacle_input_queue: queue_wrapper.QueueWrapper,
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
    if merge_data_type == "OBSTACLES":
        merge_data_type = MergeDataType.OBSTACLES
    elif merge_data_type == "DETECTIONS":
        merge_data_type = MergeDataType.DETECTIONS

    detections = []
    obstacles = []
    while not controller.is_exit_requested():
        controller.check_pause()

        if merge_data_type == MergeDataType.OBSTACLES:
            try:
                new_obstacle: obstacle.Obstacle = obstacle_input_queue.queue.get_nowait()
                obstacles.append(new_obstacle)
            except queue.Empty:
                if len(obstacles) == 0:
                    continue
                time.sleep(delay)

            try:
                odometry: drone_odometry_local.DroneOdometryLocal = (
                    odometry_input_queue.queue.get_nowait()
                )
            except queue.Empty:
                continue

            result, merged = obstacles_and_odometry.ObstaclesAndOdometry.create(
                detections, odometry
            )
            if not result:
                continue
            obstacles = []

        elif merge_data_type == MergeDataType.DETECTIONS:
            try:
                detection: lidar_detection.LidarDetection = detection_input_queue.queue.get_nowait()
                detections.append(detection)
            except queue.Empty:
                if len(detections) == 0:
                    continue
                time.sleep(delay)

            try:
                odometry: drone_odometry_local.DroneOdometryLocal = (
                    odometry_input_queue.queue.get_nowait()
                )

            except queue.Empty:
                continue

            result, merged = detections_and_odometry.DetectionsAndOdometry.create(
                detections, odometry
            )
            if not result:
                continue
            detections = []
        else:
            # log error
            return

        output_queue.queue.put(merged)
