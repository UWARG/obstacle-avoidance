from . import vector_field_histogram
from modules import lidar_oscillation
from modules import polar_obstacle_density
from worker import queue_wrapper
from worker import worker_controller


def vector_field_histogram_worker(
    sector_width: float,
    max_vector_magnitude: float,
    linear_decay_rate: float,
    confidence_value: float,
    start_angle: float,
    end_angle: float,
    oscillation_in_queue: queue_wrapper.QueueWrapper,
    density_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
) -> None:
    """ """

    # Initialize the VectorFieldHistogram
    vfh = vector_field_histogram.VectorFieldHistogram(
        sector_width,
        max_vector_magnitude,
        linear_decay_rate,
        confidence_value,
        start_angle,
        end_angle,
    )

    while not controller.is_exit_requested():
        controller.check_pause()

        oscillation: lidar_oscillation.LidarOscillation = oscillation_in_queue.queue.get()
        if lidar_oscillation is None:
            break

        result, polar_density = vfh.run(oscillation)
        if not result:
            continue

        print(f"PolarObstacleDensity sent to Decision module")
        density_out_queue.queue.put(polar_density)
