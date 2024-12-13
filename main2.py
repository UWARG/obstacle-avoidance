"""
New main
"""

import multiprocessing as mp
import pathlib
import time

import yaml

import worker.queue_wrapper
import worker.worker_controller
import modules.lidar_parser.lidar_parser_worker
import modules.vfh.vfh_worker
import modules.vfh_decision.vfh_decision_worker
import modules.detection.detection_worker
import modules.flight_interface.flight_interface_worker

CONFIG_FILE_PATH = pathlib.Path("config2.yaml")


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
        QUEUE_MAX_SIZE = config["queue_max_size"]

        # Detection parameters
        SERIAL_PORT_NAME = config["detection"]["serial_port_name"]
        SERIAL_PORT_BAUDRATE = config["detection"]["serial_port_baudrate"]
        PORT_TIMEOUT = config["detection"]["port_timeout"]
        UPDATE_RATE = config["detection"]["update_rate"]
        LOW_ANGLE = config["detection"]["low_angle"]
        HIGH_ANGLE = config["detection"]["high_angle"]
        ROTATE_SPEED = config["detection"]["rotate_speed"]

        # VFH parameters
        SECTOR_WIDTH = config["sector_width"]
        MAX_VECTOR_MAGNITUDE = config["max_vector_magnitude"]
        LINEAR_DECAY_RATE = config["linear_decay_rate"]
        CONFIDENCE_VALUE = config["confidence_value"]
        START_ANGLE = config["start_angle"]
        END_ANGLE = config["end_angle"]

        # VFH Decision parameters
        DENSITY_THRESHOLD = config["density_threshold"]
        MIN_CONSEC_SECTORS = config["min_consec_sectors"]
        WIDE_VALLEY_THRESHOLD = config["wide_valley_threshold"]

        # Flight interface parameters
        FLIGHT_INTERFACE_ADDRESS = config["flight_interface"]["address"]
        FLIGHT_INTERFACE_TIMEOUT = config["flight_interface"]["timeout"]
        FLIGHT_INTERFACE_WORKER_PERIOD = config["flight_interface"]["worker_period"]
        FIRST_WAYPOINT_DISTANCE_TOLERANCE = config["flight_interface"][
            "first_waypoint_distance_tolerance"
        ]
    except KeyError:
        print("Config key(s) not found.")
        return -1

    # Setup
    controller = worker.worker_controller.WorkerController()
    mp_manager = mp.Manager()

    detection_to_lidar_parsing_queue = worker.queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    oscillation_out_queue = worker.queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    density_out_queue = worker.queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    angle_out_queue = worker.queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)

    # Flight Interface queues
    command_to_flight_interface_queue = worker.queue_wrapper.QueueWrapper(
        mp_manager, QUEUE_MAX_SIZE
    )

    # Processes
    detection_process = mp.Process(
        target=modules.detection.detection_worker.detection_worker,
        args=(
            SERIAL_PORT_NAME,
            SERIAL_PORT_BAUDRATE,
            PORT_TIMEOUT,
            UPDATE_RATE,
            LOW_ANGLE,
            HIGH_ANGLE,
            ROTATE_SPEED,
            detection_to_lidar_parsing_queue,
            controller,
        ),
    )

    lidar_parsing_process = mp.Process(
        target=modules.lidar_parser.lidar_parser_worker.lidar_parser_worker,
        args=(
            detection_to_lidar_parsing_queue,
            oscillation_out_queue,
            controller,
        ),
    )

    vfh_process = mp.Process(
        target=modules.vfh.vfh_worker.vfh_worker,
        args=(
            SECTOR_WIDTH,
            MAX_VECTOR_MAGNITUDE,
            LINEAR_DECAY_RATE,
            CONFIDENCE_VALUE,
            START_ANGLE,
            END_ANGLE,
            oscillation_out_queue,
            density_out_queue,
            controller,
        ),
    )

    vfh_decision_process = mp.Process(
        target=modules.vfh_decision.vfh_decision_worker.vfh_decision_worker,
        args=(
            DENSITY_THRESHOLD,
            MIN_CONSEC_SECTORS,
            WIDE_VALLEY_THRESHOLD,
            density_out_queue,
            angle_out_queue,
            controller,
        ),
    )

    flight_interface_process = mp.Process(
        target=modules.flight_interface.flight_interface_worker.flight_interface_worker,
        args=(
            FLIGHT_INTERFACE_ADDRESS,
            FLIGHT_INTERFACE_TIMEOUT,
            FIRST_WAYPOINT_DISTANCE_TOLERANCE,
            FLIGHT_INTERFACE_WORKER_PERIOD,
            angle_out_queue,
            command_to_flight_interface_queue,
            controller,
        ),
    )

    # Run (mimicking main logic)
    detection_process.start()
    lidar_parsing_process.start()
    vfh_process.start()
    vfh_decision_process.start()
    flight_interface_process.start()

    while True:
        try:
            time.sleep(0.1)
        except KeyboardInterrupt:
            controller.request_exit()
            break

    # Teardown
    detection_to_lidar_parsing_queue.fill_and_drain_queue()
    oscillation_out_queue.fill_and_drain_queue()
    density_out_queue.fill_and_drain_queue()
    angle_out_queue.fill_and_drain_queue()
    command_to_flight_interface_queue.fill_and_drain_queue()

    detection_process.join()
    lidar_parsing_process.join()
    vfh_process.join()
    vfh_decision_process.join()
    flight_interface_process.join()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Error: Status Code: {result_main} ")
    print(result_main)

    print("Done.")
