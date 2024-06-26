"""
Creates decision for next action based on LiDAR detections and current odometry.
"""

from collections import deque
import enum

from .. import decision_command
from .. import detections_and_odometry


class Decision:
    """
    Determines best action to avoid obstacles based on LiDAR and odometry data.
    """

    class DroneState(enum.Enum):
        """
        Possible drone states.
        """

        STOPPED = 0
        MOVING = 1

    def __init__(self, state: DroneState, proximity_limit: float, max_history: int) -> None:
        """
        Initialize current drone state and its lidar detections list.
        """
        self.detections_and_odometries = deque(maxlen=max_history)
        self.state = state
        self.proximity_limit = proximity_limit

    def run_simple_decision(
        self,
        detections_and_odometries: "deque[detections_and_odometry.DetectionsAndOdometry]",
        proximity_limit: float,
    ) -> "tuple[bool, decision_command.DecisionCommand | None]":
        """
        Runs simple collision avoidance where drone will stop within a set distance of an object.
        """
        for lidar_scan_and_odometry in detections_and_odometries:
            detections = lidar_scan_and_odometry.detections

            if self.state == Decision.DroneState.STOPPED:
                for detection in detections:
                    if detection.distance < proximity_limit:
                        return False, None
                self.state = Decision.DroneState.MOVING
                return decision_command.DecisionCommand.create_resume_mission_command()

            for detection in detections:
                if detection.distance < proximity_limit:
                    self.state = Decision.DroneState.STOPPED
                    return decision_command.DecisionCommand.create_stop_mission_and_halt_command()
        return False, None

    def run(
        self, merged_data: detections_and_odometry.DetectionsAndOdometry
    ) -> "tuple[bool, decision_command.DecisionCommand | None]":
        """
        Run obstacle avoidance.
        """
        self.detections_and_odometries.append(merged_data)
        return self.run_simple_decision(self.detections_and_odometries, self.proximity_limit)
