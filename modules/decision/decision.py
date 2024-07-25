"""
Creates decision for next action based on LiDAR detections and current odometry.
"""

from collections import deque
import time

from .. import decision_command
from .. import detections_and_odometry
from .. import drone_odometry_local


class Decision:
    """
    Determines best action to avoid obstacles based on LiDAR and odometry data.
    """

    def __init__(self, proximity_limit: float, max_history: int, command_timeout: float) -> None:
        """
        Initialize current drone state and its lidar detections list.
        """
        self.proximity_limit = proximity_limit
        self.detections_and_odometries = deque(maxlen=max_history)
        self.command_timeout = command_timeout
        self.__command_requested = False
        self.__last_command_sent = None

    def run_simple_decision(
        self,
        detections_and_odometries: "deque[detections_and_odometry.DetectionsAndOdometry]",
        proximity_limit: float,
        current_flight_mode: drone_odometry_local.FlightMode,
    ) -> "tuple[bool, decision_command.DecisionCommand | None]":
        """
        Runs simple collision avoidance where drone will stop within a set distance of an object.
        """
        start_time = 0
        for lidar_scan_and_odometry in detections_and_odometries:
            detections = lidar_scan_and_odometry.detections

            if self.__command_requested and self.__last_command_sent == current_flight_mode:
                self.__command_requested = False

            if self.__command_requested:
                if start_time - time.time() > self.command_timeout:
                    if self.__last_command_sent == drone_odometry_local.FlightMode.STOPPED:
                        return decision_command.DecisionCommand.create_stop_mission_and_halt_command()
                    if self.__last_command_sent == drone_odometry_local.FlightMode.MOVING:
                        return decision_command.DecisionCommand.create_resume_mission_command()
                continue

            if current_flight_mode == drone_odometry_local.FlightMode.STOPPED:
                for detection in detections:
                    if detection.distance < proximity_limit:
                        return False, None
                self.__command_requested = True
                self.__last_command_sent = drone_odometry_local.FlightMode.MOVING
                self.detections_and_odometries.clear()
                start_time = time.time()
                return decision_command.DecisionCommand.create_resume_mission_command()
            
            if current_flight_mode == drone_odometry_local.FlightMode.MOVING:
                for detection in detections:
                    if detection.distance < proximity_limit:
                        self.__command_requested = True
                        self.__last_command_sent = drone_odometry_local.FlightMode.STOPPED
                        self.detections_and_odometries.clear()
                        start_time = time.time()
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
        current_flight_mode = merged_data.odometry.flight_mode
        self.detections_and_odometries.append(merged_data)
        return self.run_simple_decision(
            self.detections_and_odometries, self.proximity_limit, current_flight_mode
        )
