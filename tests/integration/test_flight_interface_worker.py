"""
Flight interface worker integration test.
"""

import multiprocessing as mp
import queue
import time

from modules import decision_command
from modules import odometry_and_waypoint
from modules.flight_interface import flight_interface_worker
from modules.common.modules import position_local
from modules.common.modules import orientation
from worker import worker_controller
from worker import queue_wrapper


# Constants
QUEUE_MAX_SIZE = 10
FLIGHT_INTERFACE_ADDRESS = "tcp:127.0.0.1:14550"
FLIGHT_INTERFACE_TIMEOUT = 10.0
FLIGHT_INTERFACE_WORKER_PERIOD = 0.1
FIRST_WAYPOINT_DISTANCE_TOLERANCE = 1.0  # metres


def simulate_decision_worker(in_queue: queue_wrapper.QueueWrapper) -> None:
    """
    Place example commands into the queue.
    """
    result, stop_command = decision_command.DecisionCommand.create_stop_mission_and_halt_command()
    assert result
    assert stop_command is not None

    in_queue.queue.put(stop_command)

    result, resume_command = decision_command.DecisionCommand.create_resume_mission_command()
    assert result
    assert resume_command is not None

    in_queue.queue.put(resume_command)


def main() -> int:
    """
    Main function.
    """
    # Setup
    controller = worker_controller.WorkerController()
    mp_manager = mp.Manager()

    command_in_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    odometry_out_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)

    worker = mp.Process(
        target=flight_interface_worker.flight_interface_worker,
        args=(
            FLIGHT_INTERFACE_ADDRESS,
            FLIGHT_INTERFACE_TIMEOUT,
            FIRST_WAYPOINT_DISTANCE_TOLERANCE,
            FLIGHT_INTERFACE_WORKER_PERIOD,
            command_in_queue,
            odometry_out_queue,
            controller,
        ),
    )

    # Run
    worker.start()

    simulate_decision_worker(command_in_queue)

    time.sleep(3)

    # Test
    while True:
        try:
            input_data: odometry_and_waypoint.OdometryAndWaypoint = (
                odometry_out_queue.queue.get_nowait()
            )
            assert (
                str(type(input_data)) == "<class 'modules.drone_odometry_local.DroneOdometryLocal'>"
            )
            assert input_data.local_position is not None
            assert input_data.drone_orientation is not None

            print("odometry:")
            print(f"north: {str(input_data.local_position.north)}")
            print(f"east: {str(input_data.local_position.east)}")
            print(f"down: {str(input_data.local_position.down)}")
            print("orientation:")
            print(f"roll: {str(input_data.drone_orientation.roll)}")
            print(f"pitch: {str(input_data.drone_orientation.pitch)}")
            print(f"yaw: {str(input_data.drone_orientation.yaw)}")
            print(f"flight mode: {str(input_data.flight_mode)}")
            print(f"timestamp: {str(input_data.timestamp)}")
            print(f"waypoint: {str(input_data.next_waypoint)}")
            print("")

        except queue.Empty:
            break

    # Teardown
    controller.request_exit()

    command_in_queue.fill_and_drain_queue()

    worker.join()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"Error: Status Code: {result_main}")

    print("Done.")
