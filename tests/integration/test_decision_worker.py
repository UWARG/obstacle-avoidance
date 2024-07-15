"""
Decision worker integration test.
"""

import multiprocessing as mp
import queue

from modules import decision_command
from modules import detections_and_odometry
from modules import drone_odometry_local
from modules import lidar_detection
from modules.common.mavlink.modules import drone_odometry
from modules.common.mavlink.modules import flight_controller
from modules.decision import decision_worker
from worker import queue_wrapper
from worker import worker_controller

# Constants
QUEUE_MAX_SIZE = 10

OBJECT_PROXIMITY_LIMIT = 5  # metres
MAX_HISTORY = 20  # readings

# pylint: disable=duplicate-code


def simulate_data_merge_worker(in_queue: queue_wrapper.QueueWrapper) -> None:
    """
    Place example merged detection into the queue.
    """
    detections = []
    for _ in range(0, 5):
        result, detection = lidar_detection.LidarDetection.create(0.0, 0.0)
        assert result
        assert detection is not None

        detections.append(detection)

    result, position = drone_odometry_local.DronePositionLocal.create(0.0, 0.0, 0.0)
    assert result
    assert position is not None

    result, orientation = drone_odometry.DroneOrientation.create(0.0, 0.0, 0.0)
    assert result
    assert orientation is not None

    flight_mode = flight_controller.FlightController.FlightMode.MOVING

    result, odometry = drone_odometry_local.DroneOdometryLocal.create(
        position, orientation, flight_mode
    )
    assert result
    assert odometry is not None

    result, merged = detections_and_odometry.DetectionsAndOdometry.create(detections, odometry)
    assert result
    assert odometry is not None

    in_queue.queue.put(merged)


def main() -> int:
    """
    Main function.
    """
    # Setup
    controller = worker_controller.WorkerController()
    mp_manager = mp.Manager()

    merged_in_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    command_out_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)

    worker = mp.Process(
        target=decision_worker.decision_worker,
        args=(
            OBJECT_PROXIMITY_LIMIT,
            MAX_HISTORY,
            merged_in_queue,
            command_out_queue,
            controller,
        ),
    )

    # Run
    worker.start()

    simulate_data_merge_worker(merged_in_queue)

    # Test
    while True:
        try:
            input_data: decision_command.DecisionCommand = command_out_queue.queue.get_nowait()

            assert input_data is not None
            assert str(type(input_data)) == "<class 'modules.decision_command.DecisionCommand'>"

            print(input_data.command)

        except queue.Empty:
            continue

    # Teardown
    controller.request_exit()

    merged_in_queue.fill_and_drain_queue()

    worker.join()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"ERROR: Status code: {result_main}")

    print("Done!")
