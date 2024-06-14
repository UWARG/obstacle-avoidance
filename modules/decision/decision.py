"""
Creates decision for next action based on LiDAR detections and current odometry.
"""


class Decision:
    """
    Determines best action to avoid obstacles based on LiDAR and odometry data.
    """

    def __init__(self):
        self.detection_and_odometries = []

    def run_simple_decision(self, detections_and_odometries, proximity_limit):
        """
        Runs simple collision avoidance where drone will stop within a set distance of an object.
        """
        for detections_and_odometry in detections_and_odometries:
            detections = detections_and_odometry.detections

            if HALTED:
                for detection in detections:
                    if detection.distance < proximity_limit:
                        return None
                    return RESUME_COMMAND
            else:
                for detection in detections:
                    if detection.distance < proximity_limit:
                        return HALT_COMMAND
        return None

    def run(self, merged_data):
        """
        Run obstacle avoidance.
        """
        result = self.run_simple_decision(merged_data, 5)
        if not result:
            return False, None
        return True, result
