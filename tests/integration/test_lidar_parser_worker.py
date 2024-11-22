"""
Integration test for lidar_parser_worker.
"""

import multiprocessing as mp
import queue

from modules import lidar_detection
from modules import lidar_oscillation
from modules.lidar_parser import lidar_parser_worker
from worker import queue_wrapper
from worker import worker_controller

# Constants
QUEUE_MAX_SIZE = 10

# pylint: disable=duplicate-code


def simulate_lidar_detection_worker(detection_in_queue: queue_wrapper.QueueWrapper) -> None:
    """
    Simulates a LiDAR detection stream and places it into the input queue.
    """
    # Simulate LiDAR detections with multiple oscillations
    for angle in range(-90, 91, 2):
        distance = 20.0 if angle < 0 else 7.5
        result, detection = lidar_detection.LidarDetection.create(distance, angle)
        assert result
        assert detection is not None
        detection_in_queue.queue.put(detection)

    for angle in range(89, -86, -2):
        distance = 7.5 if angle >= 0 else 20.0
        result, detection = lidar_detection.LidarDetection.create(distance, angle)
        assert result
        assert detection is not None
        detection_in_queue.queue.put(detection)

    for angle in range(-84, 91, 2):
        distance = 20.0 if angle < 0 else 7.5
        result, detection = lidar_detection.LidarDetection.create(distance, angle)
        assert result
        assert detection is not None
        detection_in_queue.queue.put(detection)

    for angle in range(89, 59, -6):
        result, detection = lidar_detection.LidarDetection.create(7.5, angle)
        assert result
        assert detection is not None
        detection_in_queue.queue.put(detection)
        assert result
        assert detection is not None
        detection_in_queue.queue.put(detection)

    detection_in_queue.queue.put(None)


def main() -> int:
    """
    Main function to test lidar_parser_worker.
    """
    # Setup
    controller = worker_controller.WorkerController()
    mp_manager = mp.Manager()

    detection_in_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    oscillation_out_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)

    worker = mp.Process(
        target=lidar_parser_worker.lidar_parser_worker,
        args=(detection_in_queue, oscillation_out_queue, controller),
    )

    # Run
    worker.start()

    simulate_lidar_detection_worker(detection_in_queue)

    # Test
    oscillation_count = 0
    while True:
        try:
            oscillation: lidar_oscillation.LidarOscillation = (
                oscillation_out_queue.queue.get_nowait()
            )

            assert oscillation is not None
            assert isinstance(oscillation, lidar_oscillation.LidarOscillation)

            assert len(oscillation.readings) > 0
            print(
                f"Oscillation {oscillation_count + 1} detected with {len(oscillation.readings)} readings. "
                f"Angles: {[reading.angle for reading in oscillation.readings]}"
            )

            oscillation_count += 1

        except queue.Empty:
            continue

    assert oscillation_count > 0
    print(f"Total oscillations detected: {oscillation_count}")

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
