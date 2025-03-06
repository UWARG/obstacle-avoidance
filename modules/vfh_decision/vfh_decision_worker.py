"""
Runs the VFHDecision class in a loop to process obstacle densities and output steering angles.
"""

from . import vfh_decision
from modules import odometry_and_waypoint
from modules import polar_obstacle_density
from worker import queue_wrapper
from worker import worker_controller


def vfh_decision_worker(
    density_threshold: float,
    min_consec_sectors: int,
    wide_valley_threshold: int,
    density_in_queue: queue_wrapper.QueueWrapper,
    odometry_and_waypoint_in_queue: queue_wrapper.QueueWrapper,
    angle_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """
    Processes polar obstacle densities and outputs steering angles.

    :param density_in_queue: Queue containing PolarObstacleDensity objects.
    :param command_out_queue: Queue to output steering angles.
    :param controller: Worker controller for managing worker states.
    :param density_threshold: Threshold for identifying candidate valleys.
    :param min_consec_sectors: Minimum number of consecutive sectors for a valid valley.
    :param wide_valley_threshold: Threshold to classify valleys as wide or narrow.
    """
    decision = vfh_decision.VFHDecision(
        density_threshold, min_consec_sectors, wide_valley_threshold
    )

    prev_odometry_and_waypoint_instance = None
    prev_polar_density = None

    while not controller.is_exit_requested():
        controller.check_pause()

        try:
            polar_density: polar_obstacle_density.PolarObstacleDensity = (
                density_in_queue.queue.get_nowait()
            )
        except queue_wrapper.queue.Empty:
            if prev_polar_density is None:
                continue
        polar_density = prev_polar_density

        try:
            odometry_and_waypoint_instance: odometry_and_waypoint.OdometryAndWaypoint = (
                odometry_and_waypoint_in_queue.queue.get_nowait()
            )
            prev_odometry_and_waypoint_instance = odometry_and_waypoint_instance
        except queue_wrapper.queue.Empty:
            if prev_odometry_and_waypoint_instance is None:
                continue
        odometry_and_waypoint_instance = prev_odometry_and_waypoint_instance

        result, steering_angle = decision.run(polar_density, odometry_and_waypoint_instance)
        if not result:
            continue

        angle_out_queue.queue.put(steering_angle)
        print(f"Steering angle {steering_angle} sent to angle_out_queue.")
