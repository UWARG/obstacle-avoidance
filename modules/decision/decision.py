"""
Creates decision for next action based on LiDAR detections and current odometry.
"""


class Decision:
    """
    Determines best action to avoid obstacles based on LiDAR and odometry data.
    """

    def __init__(self, state):
        self.detection_and_odometries = []
        self.state = state

    def run_simple_decision(self, detections_and_odometries, proximity_limit):
        """
        Runs simple collision avoidance where drone will stop within a set distance of an object.
        """
        for detections_and_odometry in detections_and_odometries:
            detections = detections_and_odometry.detections

            if self.state == "HALTED":
                for detection in detections:
                    if detection.distance < proximity_limit:
                        return None
                    self.state = "MOVING"
                    return "RESUME"
            else:
                for detection in detections:
                    if detection.distance < proximity_limit:
                        self.state = "HALTED"
                        return "STOP"
        return None

    def run(self, merged_data):
        """
        Run obstacle avoidance.
        """
        result = self.run_simple_decision(merged_data, 5)
        if not result:
            return False, None
        return True, result
