"""
VFH integration test.
"""

import multiprocessing as mp
import queue
import numpy as np

from modules import lidar_detection
from modules import lidar_oscillation
from modules.vfh import vfh
from worker import queue_wrapper
from worker import worker_controller

# Constants
QUEUE_MAX_SIZE = 10
SECTOR_WIDTH = 5.0
MAX_VECTOR_MAGNITUDE = 1.0
LINEAR_DECAY_RATE = 0.05
CONFIDENCE_VALUE = 1.0
START_ANGLE = -90.0
END_ANGLE = 90.0
THRESHOLD = 0.5

# pylint: disable=duplicate-code


def simulate_oscillation_worker(in_queue: queue_wrapper.QueueWrapper) -> None:
    """
    Simulate LidarOscillation and push it to the queue.
    """
    readings = []

    for angle in np.arange(-90, -30, SECTOR_WIDTH):
        result, detection = lidar_detection.LidarDetection.create(angle=angle, distance=5.0)
        assert result, "Failed to create LidarDetection"
        readings.append(detection)

    for angle in np.arange(-30, 30, SECTOR_WIDTH):
        result, detection = lidar_detection.LidarDetection.create(angle=angle, distance=100.0)
        assert result, "Failed to create LidarDetection"
        readings.append(detection)

    for angle in np.arange(30, 90, SECTOR_WIDTH):
        result, detection = lidar_detection.LidarDetection.create(angle=angle, distance=5.0)
        assert result, "Failed to create LidarDetection"
        readings.append(detection)

    result, oscillation = lidar_oscillation.LidarOscillation.create(readings)
    assert result, "Failed to create LidarOscillation"
    assert oscillation is not None

    in_queue.queue.put(oscillation)

    result, populated_oscillation = lidar_oscillation.LidarOscillation.create(
        [
            lidar_detection.LidarDetection.create(angle=angle, distance=50.0)[1]
            for angle in range(-90, 95, 5)
        ]
    )
    assert result, "Failed to create a blank LidarOscillation"
    assert populated_oscillation is not None

    in_queue.queue.put(populated_oscillation)


def main() -> int:
    """
    Main function for the VFH integration test.
    """
    # Setup
    controller = worker_controller.WorkerController()
    mp_manager = mp.Manager()

    oscillation_in_queue = queue_wrapper.QueueWrapper(mp_manager, QUEUE_MAX_SIZE)

    vfh_instance = vfh.VectorFieldHistogram(
        sector_width=SECTOR_WIDTH,
        max_vector_magnitude=MAX_VECTOR_MAGNITUDE,
        linear_decay_rate=LINEAR_DECAY_RATE,
        confidence_value=CONFIDENCE_VALUE,
        start_angle=START_ANGLE,
        end_angle=END_ANGLE,
    )

    # Simulate LidarOscillation data
    simulate_oscillation_worker(oscillation_in_queue)

    # Test
    density_count = 0
    while True:
        try:
            input_data: lidar_oscillation.LidarOscillation = oscillation_in_queue.queue.get_nowait()
            assert input_data is not None

            result, output_data = vfh_instance.run(input_data)
            assert result, "VFH computation failed"
            assert output_data is not None

            # Print detailed debug information
            print(f"PolarObstacleDensity {density_count + 1}:")
            print(f"  Sectors: {len(output_data.sector_densities)}")
            print(
                f"  Sector Densities: {[sector.density for sector in output_data.sector_densities]}"
            )
            print(f"  Angles: {[sector.angle_start for sector in output_data.sector_densities]}")

            density_count += 1

        except queue.Empty:
            break

    assert density_count > 0, "No PolarObstacleDensity objects were processed"
    print(f"Total PolarObstacleDensities processed: {density_count}")

    # Teardown
    controller.request_exit()
    oscillation_in_queue.fill_and_drain_queue()

    return 0


if __name__ == "__main__":
    result_main = main()
    if result_main < 0:
        print(f"ERROR: Status code: {result_main}")

    print("Done!")
