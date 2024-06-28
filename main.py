"""
Main
"""

import multiprocessing as mp
import time

from modules.data_merge import data_merge_worker
from modules.decision import decision
from modules.decision import decision_worker
from modules.detection import detection_worker
from modules.flight_interface import flight_interface_worker
from worker import queue_wrapper
from worker import worker_controller


def main() -> int:
    """
    Main function
    """

    # Local constants
    # pylint: disable=invalid-name
    QUEUE_MAX_SIZE = 10

    FLIGHT_INTERFACE_ADDRESS = "/dev/ttyUSB0"
    FLIGHT_INTERFACE_TIMEOUT = 10.0
    FLIGHT_INTERFACE_WORKER_PERIOD = 0.1

    SERIAL_PORT_NAME = "/dev/ttyACM0"
    SERIAL_PORT_BAUDRATE = 921600
    PORT_TIMEOUT = 0.1  # seconds
    UPDATE_RATE = 5
    HIGH_ANGLE = 170
    LOW_ANGLE = -170
    ROTATE_SPEED = 5

    DELAY = 0.1

    INITIAL_DRONE_STATE = decision.Decision.DroneState.MOVING
    OBJECT_PROXIMITY_LIMIT = 10  # metres
    MAX_HISTORY = 20  # readings
    # pylint: enable=invalid-name

    controller = worker_controller.WorkerController()
    mp_manager = mp.Manager()

    flight_interface_to_data_merge_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    detection_to_data_merge_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    merged_to_decision_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    command_to_flight_interface_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)

    flight_interface_process = mp.Process(
        target=flight_interface_worker.flight_interface_worker,
        args=(
            FLIGHT_INTERFACE_ADDRESS,
            FLIGHT_INTERFACE_TIMEOUT,
            FLIGHT_INTERFACE_WORKER_PERIOD,
            command_to_flight_interface_queue,
            flight_interface_to_data_merge_queue,
            controller,
        ),
    )

    detection_process = mp.Process(
        target=detection_worker.detection_worker,
        args=(
            SERIAL_PORT_NAME,
            SERIAL_PORT_BAUDRATE,
            PORT_TIMEOUT,
            UPDATE_RATE,
            HIGH_ANGLE,
            LOW_ANGLE,
            ROTATE_SPEED,
            detection_to_data_merge_queue,
            controller,
        ),
    )

    data_merge_process = mp.Process(
        target=data_merge_worker.data_merge_worker,
        args=(
            DELAY,
            detection_to_data_merge_queue,
            flight_interface_to_data_merge_queue,
            merged_to_decision_queue,
            controller,
        ),
    )

    decision_process = mp.Process(
        target=decision_worker.decision_worker,
        args=(
            INITIAL_DRONE_STATE,
            OBJECT_PROXIMITY_LIMIT,
            MAX_HISTORY,
            merged_to_decision_queue,
            command_to_flight_interface_queue,
            controller,
        ),
    )

    flight_interface_process.start()
    detection_process.start()
    data_merge_process.start()
    decision_process.start()

    while True:
        try:
            time.sleep(0.1)

        except KeyboardInterrupt:
            controller.request_exit()
            break

    flight_interface_to_data_merge_queue.fill_and_drain_queue()
    detection_to_data_merge_queue.fill_and_drain_queue()
    merged_to_decision_queue.fill_and_drain_queue()
    command_to_flight_interface_queue.fill_and_drain_queue()

    flight_interface_process.join()
    detection_process.join()
    data_merge_process.join()
    decision_process.join()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Error: Status Code: {result_main} ")
    print(result_main)

    print("Done.")
