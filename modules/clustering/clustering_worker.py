"""
Gets detection clusters.
"""

import queue

from modules import lidar_detection
from worker import queue_wrapper
from worker import worker_controller
from . import clustering


def clustering_worker(
    max_cluster_distance: float,
    detection_in_queue: queue_wrapper.QueueWrapper,
    cluster_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    max_cluster_distance: max distance between points in the same cluster in metres.
    """
    clusterer = clustering.Clustering(max_cluster_distance)

    while not controller.is_exit_requested():
        controller.check_pause()

        try:
            detection: lidar_detection.LidarDetection = detection_in_queue.queue.get_nowait()
            if detection is None:
                break
        except queue.Empty:
            continue

        result, value = clusterer.run(detection)
        if not result:
            continue

        cluster_out_queue.queue.put(value)
