"""
Clustering worker integration test.
"""

import multiprocessing as mp
import queue
import random
import time

from modules import detection_cluster
from modules import lidar_detection
from modules.clustering import clustering_worker
from worker import queue_wrapper
from worker import worker_controller

# Constants
QUEUE_MAX_SIZE = 10
DELAY = 0.1
MAX_CLUSTER_DISTANCE = 3.0  # metres


def simulate_detection_worker(in_queue: queue_wrapper.QueueWrapper) -> None:
    """
    Place example lidar reading into the queue.
    """
    random_distance = random.randint(0, 50)
    result, detection = lidar_detection.LidarDetection.create(random_distance, 0.0)
    assert result
    assert detection is not None

    in_queue.queue.put(detection)


def main() -> int:
    """
    Main function.
    """

    # Setup
    controller = worker_controller.WorkerController()
    mp_manager = mp.Manager()

    detection_in_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    cluster_out_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)

    worker = mp.Process(
        target=clustering_worker.clustering_worker,
        args=(
            MAX_CLUSTER_DISTANCE,
            detection_in_queue,
            cluster_out_queue,
            controller,
        ),
    )

    # Run
    worker.start()

    while True:
        simulate_detection_worker(detection_in_queue)
        try:
            input_data: detection_cluster.DetectionCluster = cluster_out_queue.queue.get_nowait()

            assert input_data is not None
            assert str(type(input_data)) == "<class 'modules.detection_cluster.DetectionCluster'>"

            print(input_data)

        except queue.Empty:
            time.sleep(DELAY)
            continue

        time.sleep(DELAY)

    # Teardown
    controller.request_exit()

    detection_in_queue.fill_and_drain_queue()

    worker.join()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"ERROR: Status code: {result_main}")

    print("Done!")
