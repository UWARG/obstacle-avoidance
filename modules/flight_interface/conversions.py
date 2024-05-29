import pymap3d as pymap

from .. import drone_odometry_local
from ..common.mavlink.modules import drone_odometry


def position_global_to_local(
    global_position: drone_odometry.DronePosition, home_location: drone_odometry.DronePosition
) -> "tuple[bool, drone_odometry_local.DronePositionLocal | None]":
    """
    Converts global position (geodetic) to local position (NED)
    """
    north, east, down = pymap.geodetic2ned(
        global_position.latitude,
        global_position.longitude,
        global_position.altitude,
        home_location.latitude,
        home_location.longitude,
        home_location.altitude,
    )

    result, local_position = drone_odometry_local.DronePositionLocal.create(north, east, down)
    if not result:
        return False, None

    return True, local_position


def position_local_to_global(
    local_position: drone_odometry_local.DronePositionLocal,
    home_location: drone_odometry.DronePosition,
) -> "tuple[bool, drone_odometry.DronePosition | None]":
    """
    Converts local position (NED) to global position (geodetic)
    """
    latitude, longitude, altitude = pymap.ned2geodetic(
        local_position.north,
        local_position.east,
        local_position.down,
        home_location.latitude,
        home_location.longitude,
        home_location.altitude,
    )

    result, global_position = drone_odometry.DronePosition.create(latitude, longitude, altitude)
    if not result:
        return False, None

    return True, global_position
