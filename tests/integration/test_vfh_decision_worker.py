"""
Integration test for VFHDecisionWorker.
"""

import multiprocessing as mp
import queue

from modules import odometry_and_waypoint
from modules import polar_obstacle_density
from worker import queue_wrapper
from worker import worker_controller
from modules.vfh_decision import vfh_decision_worker
from modules.common.modules import position_local
from modules.common.modules import orientation

# Constants
QUEUE_MAX_SIZE = 10
DENSITY_THRESHOLD = 0.5
MIN_CONSEC_SECTORS = 3
WIDE_VALLEY_THRESHOLD = 6


def simulate_input_queues(
    density_in_queue: queue_wrapper.QueueWrapper,
    odometry_in_queue: queue_wrapper.QueueWrapper,
) -> None:
    """
    Simulates input data for both PolarObstacleDensity and OdometryAndWaypoint.
    """
    # Simulate PolarObstacleDensity with open space
    densities = [0.0] * 36
    sectors = []
    for i, density in enumerate(densities):
        angle_start = -90 + i * 5
        angle_end = angle_start + 5
        result, sector = polar_obstacle_density.SectorObstacleDensity.create(
            angle_start, angle_end, density
        )
        assert result
        sectors.append(sector)

    result, polar_density = polar_obstacle_density.PolarObstacleDensity.create(sectors)
    assert result
    density_in_queue.queue.put(polar_density)

    # Simulate OdometryAndWaypoint
    result, drone_pos = position_local.PositionLocal.create(0.0, 0.0, 0.0)
    assert result

    result, waypoint_pos = position_local.PositionLocal.create(10.0, 0.0, 0.0)
    assert result

    result, drone_orientation = orientation.Orientation.create(0.0, 0.0, 0.0)
    assert result

    result, odometry_wp = odometry_and_waypoint.OdometryAndWaypoint.create(
        drone_pos,
        drone_orientation,
        odometry_and_waypoint.FlightMode.AUTO,
        waypoint_pos,
    )
    assert result
    odometry_in_queue.queue.put(odometry_wp)

    densities = [0.9] * 36  # 36 sectors for -90° to +90°, all with high density
    sectors = []

    for i, density in enumerate(densities):
        angle_start = -90 + i * 5
        angle_end = angle_start + 5
        result, sector = polar_obstacle_density.SectorObstacleDensity.create(
            angle_start, angle_end, density
        )
        assert result, f"Failed to create sector for angles {angle_start} to {angle_end}"
        sectors.append(sector)

    result, polar_density = polar_obstacle_density.PolarObstacleDensity.create(sectors)
    assert result, "Failed to create PolarObstacleDensity object"

    density_in_queue.queue.put(polar_density)


def main() -> int:
    """
    Main function to test VFHDecisionWorker.
    """
    # Setup
    controller = worker_controller.WorkerController()
    mp_manager = mp.Manager()

    density_in_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    odometry_in_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)
    angle_out_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)

    worker = mp.Process(
        target=vfh_decision_worker,
        args=(
            DENSITY_THRESHOLD,
            MIN_CONSEC_SECTORS,
            WIDE_VALLEY_THRESHOLD,
            density_in_queue,
            odometry_in_queue,
            angle_out_queue,
            controller,
        ),
    )

    # Run
    worker.start()

    simulate_input_queues(density_in_queue, odometry_in_queue)

    # Test
    steering_angle_count = 0
    while True:
        try:
            steering_angle = angle_out_queue.queue.get_nowait()
            assert (
                steering_angle is None if steering_angle_count == 0 else steering_angle is not None
            )
            print(f"Steering angle {steering_angle} received.")

            steering_angle_count += 1

        except queue.Empty:
            if controller.is_exit_requested():
                break

    assert steering_angle_count > 0
    print(f"Total steering angles computed: {steering_angle_count}")

    # Teardown
    controller.request_exit()
    density_in_queue.fill_and_drain_queue()
    odometry_in_queue.fill_and_drain_queue()
    worker.join()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"ERROR: Status code: {result_main}")

    print("Done!")
