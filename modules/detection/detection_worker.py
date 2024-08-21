"""
Gets lidar readings.
"""

from worker import queue_wrapper
from worker import worker_controller
from . import detection


def detection_worker(
    serial_port_name: str,
    serial_port_baudrate: int,
    port_timeout: float,
    update_rate: int,
    low_angle: float,
    high_angle: float,
    rotate_speed: int,
    detection_to_clustering_queue: queue_wrapper.QueueWrapper,
    detection_to_data_merge_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.

    serial_port_name: port that the lidar is connected to
    serial_port_baudrate: baudrate of the lidar port
    port_timeout: timeout for connecting to serial port
    update_rate: frequency the lidar reads points (must be integer between 1 and 12 inclusive)
    low_angle: lidar low angle in degrees (must be between -170 and -5 inclusive)
    high_angle: lidar high angle in degrees (must be between 5 and 170 inclusive)
    rotate_speed: lidar rotational speed (must be integer between 5 and 2000 inclusive where 5 is the fastest)
    """
    detector = detection.Detection(
        serial_port_name,
        serial_port_baudrate,
        port_timeout,
        update_rate,
        low_angle,
        high_angle,
        rotate_speed,
    )

    while not controller.is_exit_requested():
        controller.check_pause()

        result, value = detector.run()
        if not result:
            continue

        detection_to_clustering_queue.queue.put(value)
        detection_to_data_merge_queue.queue.put(value)
