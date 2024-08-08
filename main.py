"""
Main
"""

import multiprocessing as mp
import pathlib
import time

import yaml

from modules.data_merge import data_merge_worker
from modules.decision import decision_worker
from modules.detection import detection_worker
from modules.flight_interface import flight_interface_worker
from worker import queue_wrapper
from worker import worker_controller

CONFIG_FILE_PATH = pathlib.Path("config.yaml")


def main() -> int:
    """
    Main function
    """
    # Open config file
    try:
        with CONFIG_FILE_PATH.open("r", encoding="utf8") as file:
            try:
                config = yaml.safe_load(file)
            except yaml.YAMLError as exc:
                print(f"Error parsing YAML file: {exc}")
                return -1
    except FileNotFoundError:
        print(f"File not found: {CONFIG_FILE_PATH}")
        return -1
    except IOError as exc:
        print(f"Error when opening file: {exc}")
        return -1

    # Set constants
    try:
        # Local constants
        # pylint: disable=invalid-name
        QUEUE_MAX_SIZE = config["queue_max_size"]

        FLIGHT_INTERFACE_ADDRESS = config["flight_interface"]["address"]
        FLIGHT_INTERFACE_TIMEOUT = config["flight_interface"]["timeout"]
        FLIGHT_INTERFACE_WORKER_PERIOD = config["flight_interface"]["worker_period"]
        FIRST_WAYPOINT_DISTANCE_TOLERANCE = config["flight_interface"][
            "first_waypoint_distance_tolerance"
        ]

        SERIAL_PORT_NAME = config["detection"]["serial_port_name"]
        SERIAL_PORT_BAUDRATE = config["detection"]["serial_port_baudrate"]
        PORT_TIMEOUT = config["detection"]["port_timeout"]
        UPDATE_RATE = config["detection"]["update_rate"]
        LOW_ANGLE = config["detection"]["low_angle"]
        HIGH_ANGLE = config["detection"]["high_angle"]
        ROTATE_SPEED = config["detection"]["rotate_speed"]

        DELAY = config["data_merge"]["delay"]

        OBJECT_PROXIMITY_LIMIT = config["decision"]["object_proximity_limit"]
        MAX_HISTORY = config["decision"]["max_history"]
        COMMAND_TIMEOUT = config["decision"]["command_timeout"]
        # pylint: enable=invalid-name
    except KeyError:
        print("Config key(s) not found.")
        return -1

    # Setup
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
            FIRST_WAYPOINT_DISTANCE_TOLERANCE,
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
            LOW_ANGLE,
            HIGH_ANGLE,
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
            OBJECT_PROXIMITY_LIMIT,
            MAX_HISTORY,
            COMMAND_TIMEOUT,
            merged_to_decision_queue,
            command_to_flight_interface_queue,
            controller,
        ),
    )

    # Run
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

    # Teardown
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
