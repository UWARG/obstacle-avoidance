"""
Creates decision for next action based on LiDAR detections and current odometry.
"""

from collections import deque

from .. import decision_command
from .. import detections_and_odometry
from ..common.mavlink.modules import flight_controller


class Decision:
    """
    Determines best action to avoid obstacles based on LiDAR and odometry data.
    """

    def __init__(self, proximity_limit: float, max_history: int) -> None:
        """
        Initialize current drone state and its lidar detections list.
        """
        self.proximity_limit = proximity_limit
        self.detections_and_odometries = deque(maxlen=max_history)

    def run_simple_decision(
        self,
        detections_and_odometries: "deque[detections_and_odometry.DetectionsAndOdometry]",
        proximity_limit: float,
    ) -> "tuple[bool, decision_command.DecisionCommand | None]":
        """
        Runs simple collision avoidance where drone will stop within a set distance of an object.
        """
        for lidar_scan_and_odometry in detections_and_odometries:
            current_flight_mode = lidar_scan_and_odometry.odometry.flight_mode
            detections = lidar_scan_and_odometry.detections

            if current_flight_mode == flight_controller.FlightController.FlightMode.STOPPED:
                for detection in detections:
                    if detection.distance < proximity_limit:
                        return False, None
                return decision_command.DecisionCommand.create_resume_mission_command()
            if current_flight_mode == flight_controller.FlightController.FlightMode.MOVING:
                for detection in detections:
                    if detection.distance < proximity_limit:
                        return (
                            decision_command.DecisionCommand.create_stop_mission_and_halt_command()
                        )
        return False, None

    def run(
        self, merged_data: detections_and_odometry.DetectionsAndOdometry
    ) -> "tuple[bool, decision_command.DecisionCommand | None]":
        """
        Run obstacle avoidance.
        """
        self.detections_and_odometries.append(merged_data)
        return self.run_simple_decision(self.detections_and_odometries, self.proximity_limit)
