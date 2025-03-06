"""
Processes VFH data to compute optimal steering directions.
"""

import math

from modules import odometry_and_waypoint
from modules import polar_obstacle_density


class VFHDecision:
    """
    Computes steering direction from polar obstacle densities.
    """

    def __init__(
        self,
        density_threshold: float,
        min_consec_sectors: int,
        wide_valley_threshold: int,
    ) -> None:
        """
        Initializes VFHDecision with density threshold and valley parameters.

        :param density_threshold: The threshold for identifying candidate valleys.
        :param min_consec_sectors: Minimum number of consecutive sectors for a valid valley.
        :param wide_valley_threshold: Number of sectors distinguishing wide valleys from narrow ones.
        """
        self.density_threshold = density_threshold
        self.min_consec_sectors = min_consec_sectors
        self.wide_valley_threshold = wide_valley_threshold

    def run(
        self,
        polar_density: polar_obstacle_density.PolarObstacleDensity,
        odometry_waypoint: odometry_and_waypoint.OdometryAndWaypoint,
    ) -> "tuple[bool, float | None | str]":
        """
        Identifies valleys and computes the steering angle.

        :param polar_density: PolarObstacleDensity object containing sector densities.
        :param odometry_waypoint: OdometryAndWaypoint object containing drone's current odometry, flight mode, and next wp info.
        :return: Tuple (success: bool, steering_angle: float | None).
        """

        # Compute target angle from drone's current position and next waypoint
        drone_pos = odometry_waypoint.local_position
        waypoint_pos = odometry_waypoint.next_waypoint
        delta_north = waypoint_pos.north - drone_pos.north
        delta_east = waypoint_pos.east - drone_pos.east
        target_angle = math.degrees(math.atan2(delta_east, delta_north))

        candidate_valleys = []
        current_valley = []

        for sector in polar_density.sector_densities:
            if sector.density < self.density_threshold:
                current_valley.append(sector)
            elif current_valley:
                print(
                    f"Identified Valley: Start={current_valley[0].angle_start}, End={current_valley[-1].angle_end}"
                )
                if len(current_valley) >= self.min_consec_sectors:
                    candidate_valleys.append(current_valley)
                current_valley = []

        if current_valley and len(current_valley) >= self.min_consec_sectors:
            candidate_valleys.append(current_valley)

        best_valley = None
        min_distance = float("inf")

        # Wide Valley Pre-Check for AUTO mode
        if odometry_waypoint.flight_mode == odometry_and_waypoint.FlightMode.AUTO:
            for valley in candidate_valleys:
                near_angle = valley[0].angle_start
                far_angle = valley[-1].angle_end

                # Ensure the valley contains 0° and has enough space on both sides
                if (
                    near_angle <= -self.wide_valley_threshold / 2
                    and far_angle >= self.wide_valley_threshold / 2
                ):
                    return False, None  # No need for obstacle avoidance

        for valley in candidate_valleys:
            near_angle = valley[0].angle_start
            far_angle = valley[-1].angle_end
            center_angle = (near_angle + far_angle) / 2
            print(f"Candidate Valley: Start={near_angle}, End={far_angle}, Center={center_angle}")

            distance = abs(center_angle - target_angle)
            if distance < min_distance:
                min_distance = distance
                best_valley = (near_angle, far_angle)

        # Total Blockage Handling
        # currently just REVERSE, meant to be an extreme flag that completely halts the drone asap, rather than deflect
        if not best_valley:
            return True, "REVERSE"

        near_angle, far_angle = best_valley
        steering_angle = (near_angle + far_angle) / 2
        return True, steering_angle
