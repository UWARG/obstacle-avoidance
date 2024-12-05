"""
Processes VFH data to compute optimal steering directions.
"""

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
        target_angle: float,
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
        self.target_angle = target_angle

    def run(
        self, polar_density: polar_obstacle_density.PolarObstacleDensity
    ) -> "tuple[bool, float | None | str]":
        """
        Identifies valleys and computes the steering angle.

        :param polar_density: PolarObstacleDensity object containing sector densities.
        :param target_angle: Desired direction of travel.
        :return: Tuple (success: bool, steering_angle: float | None).
        """
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

        # Wide Valley Pre-Check
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

            distance = abs(center_angle - self.target_angle)
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
