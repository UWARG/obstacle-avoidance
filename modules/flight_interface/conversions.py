"""
Drone position conversions to and from local (NED) and global (geodetic) space.
"""

import pymap3d as pymap

from ..common.modules import position_global
from ..common.modules import position_local


def position_global_to_local(
    global_position: position_global.PositionGlobal, home_location: position_global.PositionGlobal
) -> "tuple[bool, position_local.PositionLocal.DronePositionLocal | None]":
    """
    Converts global position (geodetic) to local position (NED).
    """
    north, east, down = pymap.geodetic2ned(
        global_position.latitude,
        global_position.longitude,
        global_position.altitude,
        home_location.latitude,
        home_location.longitude,
        home_location.altitude,
    )

    result, local_position = position_local.PositionLocal.create(north, east, down)
    if not result:
        return False, None

    return True, local_position


def position_local_to_global(
    local_position: position_local.PositionLocal,
    home_location: position_global.PositionGlobal,
) -> "tuple[bool, position_global.PositionGlobal | None]":
    """
    Converts local position (NED) to global position (geodetic).
    """
    latitude, longitude, altitude = pymap.ned2geodetic(
        local_position.north,
        local_position.east,
        local_position.down,
        home_location.latitude,
        home_location.longitude,
        home_location.altitude,
    )

    result, global_position = position_global.PositionGlobal.create(latitude, longitude, altitude)
    if not result:
        return False, None

    return True, global_position
