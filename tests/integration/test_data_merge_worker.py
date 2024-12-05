"""
Data merge worker integration test.
"""

import multiprocessing as mp
import queue
import time

from modules import detections_and_odometry
from modules import odometry_and_waypoint
from modules import lidar_detection
from modules.common.modules import position_local
from modules.common.modules import orientation
from modules.data_merge import data_merge_worker
from worker import queue_wrapper
from worker import worker_controller


# Constants
QUEUE_MAX_SIZE = 10
DELAY = 0.1  # seconds


def simulate_detection_worker(in_queue: queue_wrapper.QueueWrapper, identifier: int) -> None:
    """
    Place example lidar reading into the queue.
    """
    result, detection = lidar_detection.LidarDetection.create(float(identifier), 0.0)
    assert result
    assert detection is not None

    print("simulated detection: " + str(identifier))
    in_queue.queue.put(detection)


def simulate_flight_interface_worker(in_queue: queue_wrapper.QueueWrapper, identifier: int) -> None:
    """
    Place example odometry into the queue.
    """
    result, position = position_local.PositionLocal.create(float(identifier), 0.0, 0.0)
    assert result
    assert position is not None

    result, orientation_instance = orientation.Orientation.create(0.0, 0.0, 0.0)
    assert result
    assert orientation_instance is not None

    flight_mode = odometry_and_waypoint.OdometryAndWaypoint.FlightMode.AUTO

    next_waypoint_local = position_local.PositionLocal.create(0.0, 0.0, 0.0)

    result, odometry = odometry_and_waypoint.OdometryAndWaypoint.create(
        position, orientation_instance, flight_mode, next_waypoint_local
    )
    assert result
    assert odometry is not None

    print("simulated flight interface: " + str(identifier))
    in_queue.queue.put(odometry)


def main() -> int:
    """
    Main function.
    """
    # Setup
    controller = worker_controller.WorkerController()
    mp_manager = mp.Manager()

    detection_in_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    odometry_in_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    data_merge_out_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)

    detection_count = 0  # number of detections created (via simulation)
    odometry_count = 0  # number of odometries created (via simulation)

    worker = mp.Process(
        target=data_merge_worker.data_merge_worker,
        args=(
            DELAY,
            detection_in_queue,
            odometry_in_queue,
            data_merge_out_queue,
            controller,
        ),
    )

    # Run
    worker.start()
    print("worker started")

    # simulating 1 lidar reading and 1 odometry reading
    detection_count += 1
    simulate_detection_worker(detection_in_queue, detection_count)

    time.sleep(DELAY)  # simulating flight interface processing time
    odometry_count += 1
    simulate_flight_interface_worker(odometry_in_queue, odometry_count)

    # simulating 5 lidar readings for each odometry reading
    for _ in range(0, 5):
        detection_count += 1
        simulate_detection_worker(detection_in_queue, detection_count)

    time.sleep(DELAY)  # simulating flight interface processing time
    odometry_count += 1
    simulate_flight_interface_worker(odometry_in_queue, odometry_count)

    # Test
    while True:
        try:
            input_data: detections_and_odometry.DetectionsAndOdometry = (
                data_merge_out_queue.queue.get_nowait()
            )

            assert input_data is not None
            assert (
                str(type(input_data))
                == "<class 'modules.detections_and_odometry.DetectionsAndOdometry'>"
            )

            print("received detection and odometry")
            print(input_data)

        except queue.Empty:
            print("queue is empty")
            time.sleep(DELAY)

        # Teardown
        controller.request_exit()

        detection_in_queue.fill_and_drain_queue()
        odometry_in_queue.fill_and_drain_queue()

        worker.join()

        return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"ERROR: Status code: {result_main}")

    print("Done!")
