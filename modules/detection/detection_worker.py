"""
Gets drone odometry and combines with lidar readings.
"""

from worker import queue_wrapper
from worker import worker_controller
from . import detection


def detection_worker(
    serial_port_name: str,
    serial_port_baudrate: int,
    port_timeout: float,
    update_rate: int,
    high_angle: float,
    low_angle: float,
    rotate_speed: int,
    input_queue: queue_wrapper.QueueWrapper,
    output_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Worker process.
    """
    detector = detection.Detection(
        serial_port_name,
        serial_port_baudrate,
        port_timeout,
        update_rate,
        high_angle,
        low_angle,
        rotate_speed,
    )

    while not controller.is_exit_requested():
        controller.check_pause()

        input_data = input_queue.queue.get()
        if input_data is None:
            break

        result, value = detector.run(input_data)
        if not result:
            continue

        output_queue.queue.put(value)
