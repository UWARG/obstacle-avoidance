from . import vector_field_histogram
from modules import lidar_oscillation
from modules import PolarObstacleDensity
from worker import queue_wrapper
from worker import worker_controller


def lidar_density_worker(
    oscillation_in_queue: queue_wrapper.QueueWrapper,
    density_out_queue: queue_wrapper.QueueWrapper,
    controller: worker_controller.WorkerController,
    sector_width: float,
) -> None:
    """ """

    # Initialize the VectorFieldHistogram
    vfh = vector_field_histogram.VectorFieldHistogram(
        sector_width=2.0,
        max_vector_magnitude=1.0,
        linear_decay_rate=0.1,
        confidence_value=0.9,
        start_angle=-90.0,
        end_angle=90.0,
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
