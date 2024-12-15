"""
Gets a cluster and outputs an obstacle.
"""

from worker import queue_wrapper
from worker import worker_controller
from . import cluster_classification


def cluster_classification_worker(
    cluster_in_queue: queue_wrapper.QueueWrapper,
    obstacle_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
):
    """
    Worker process.
    """
    classifier = cluster_classification.ClusterClassification()

    while not controller.is_exit_requested():
        controller.check_pause()

        cluster: detection_cluster.DetectionCluster = cluster_in_queue.queue.get_nowait()
        if cluster is None:
            break

        result, value = classifier.run(cluster)
        if not result:
            continue

        obstacle_out_queue.queue.put(value)
