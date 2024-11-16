# pylint: skip-file
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
from modules.common.mavlink.dronekit import connect, VehicleMode, LocationGlobalRelative, Command
import time
import math
from pymavlink import mavutil
import sys
import signal

# flake8: noqa

# Global flag to indicate mission should be aborted
mission_abort = False

# Set up option parsing to get connection string
import argparse

parser = argparse.ArgumentParser(description="Mission with intermediate waypoint in GUIDED mode.")
parser.add_argument("--connect", help="Vehicle connection target string.")
args = parser.parse_args()

connection_string = args.connect if args.connect else "tcp:127.0.0.1:14550"
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl

    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print("Connecting to vehicle on: %s" % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):  # type: ignore
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude:", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def get_location_metres(original_location, dNorth, dEast):  # type: ignore
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)


def create_square_mission(aLocation, aSize):  # type: ignore
    cmds = vehicle.commands
    cmds.clear()

    # First waypoint: takeoff altitude
    cmds.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,
            0,
            0,
            0,
            0,
            0,
            aLocation.lat,
            aLocation.lon,
            10,
        )
    )

    # Define square waypoints
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)

    # Add the square waypoints and a final one back to the starting point
    cmds.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0,
            0,
            0,
            0,
            0,
            point1.lat,
            point1.lon,
            10,
        )
    )
    cmds.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0,
            0,
            0,
            0,
            0,
            point2.lat,
            point2.lon,
            10,
        )
    )
    cmds.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0,
            0,
            0,
            0,
            0,
            point3.lat,
            point3.lon,
            10,
        )
    )
    cmds.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0,
            0,
            0,
            0,
            0,
            point4.lat,
            point4.lon,
            10,
        )
    )
    cmds.add(
        Command(
            0,
            0,
            0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,
            0,
            0,
            0,
            0,
            0,
            aLocation.lat,
            aLocation.lon,
            10,
        )
    )

    cmds.upload()


def goto_position_target_global_int(aLocation):  # type: ignore
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,
        0,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # type_mask (only positions enabled)
        int(aLocation.lat * 1e7),
        int(aLocation.lon * 1e7),
        aLocation.alt,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )
    vehicle.send_mavlink(msg)


def distance_to_waypoint(current_location, target_location):  # type: ignore
    if isinstance(target_location, Command):
        target_lat = target_location.x  # Use x, y for lat, lon in Command
        target_lon = target_location.y
        target_alt = target_location.z
    else:
        target_lat = target_location.lat
        target_lon = target_location.lon
        target_alt = target_location.alt

    dlat = target_lat - current_location.lat
    dlong = target_lon - current_location.lon
    return math.sqrt((dlat * dlat) + (dlong * dlong)) * 1.113195e5


def set_loiter_mode(signal, frame):  # type: ignore
    global mission_abort
    print("\nInterrupt signal received! Aborting mission and switching to LOITER mode.")
    mission_abort = True

    if vehicle and vehicle.is_armable:
        try:
            # Force switch to LOITER mode
            vehicle.mode = VehicleMode("LOITER")

            # Confirm the mode change
            while vehicle.mode.name != "LOITER":
                print(" Waiting for LOITER mode to activate...")
                vehicle.mode = VehicleMode("LOITER")
                time.sleep(1)

            print("Successfully switched to LOITER mode and killed script")

        except Exception as e:
            print(f"Failed to switch to LOITER mode: {e}")
    else:
        print("Vehicle is not armable or not connected.")
    sys.exit(0)


# Assign signal handlers
signal.signal(signal.SIGINT, set_loiter_mode)
signal.signal(signal.SIGTERM, set_loiter_mode)


def execute_mission():  # type: ignore
    takeoff_altitude = 10

    # Arm and take off to the target altitude
    arm_and_takeoff(takeoff_altitude)

    # Store home location at the start
    home_location = vehicle.location.global_relative_frame
    print("Home location set at:", home_location)

    # Create the square mission
    create_square_mission(vehicle.location.global_relative_frame, 10)

    print("Starting mission")
    vehicle.mode = VehicleMode("AUTO")
    vehicle.commands.next = 0

    # Flag to track if we have visited the intermediate waypoint
    intermediate_waypoint_reached = False

    while not mission_abort:
        nextwaypoint = vehicle.commands.next
        print(
            "Distance to waypoint (%s): %s" % (nextwaypoint, vehicle.location.global_relative_frame)
        )

        # Check for the second waypoint to trigger intermediate waypoint diversion
        if nextwaypoint == 2 and not intermediate_waypoint_reached:
            waypoint_cmd = vehicle.commands[1]
            if distance_to_waypoint(vehicle.location.global_relative_frame, waypoint_cmd) < 25:
                print("Switching to GUIDED mode for intermediate waypoint")
                vehicle.mode = VehicleMode("GUIDED")

                # Calculate intermediate waypoint offset
                intermediate_wp = get_location_metres(
                    vehicle.location.global_relative_frame, 7.5, 7.5
                )
                goto_position_target_global_int(intermediate_wp)

                # Wait until we reach the intermediate waypoint
                while (
                    distance_to_waypoint(vehicle.location.global_relative_frame, intermediate_wp)
                    > 1
                ):
                    print(
                        " Distance to intermediate waypoint:",
                        distance_to_waypoint(
                            vehicle.location.global_relative_frame, intermediate_wp
                        ),
                    )
                    time.sleep(1)

                print("Reached intermediate waypoint, resuming AUTO mode")
                vehicle.mode = VehicleMode("AUTO")
                vehicle.commands.next = 2  # Ensure mission continues from waypoint 2
                intermediate_waypoint_reached = True

        # Check if we reached the final waypoint (5) to return to home and land
        if nextwaypoint == 5:
            print("Mission complete, returning to home location")

            # Switch to GUIDED mode to fly back to the home location explicitly
            vehicle.mode = VehicleMode("GUIDED")
            goto_position_target_global_int(home_location)

            # Wait until we reach the home location
            while distance_to_waypoint(vehicle.location.global_relative_frame, home_location) > 1:
                print(
                    " Distance to home location:",
                    distance_to_waypoint(vehicle.location.global_relative_frame, home_location),
                )
                time.sleep(1)

            # Initiate landing sequence
            print("Reached home location, initiating landing")
            vehicle.mode = VehicleMode("LAND")

            # Wait until the vehicle has landed
            while vehicle.location.global_relative_frame.alt > 0.1:
                print(
                    " Altitude during landing:",
                    vehicle.location.global_relative_frame.alt,
                )
                time.sleep(1)

            print("Landed successfully")
            break

        time.sleep(1)

    print("Mission completed, closing vehicle connection.")
    vehicle.close()


# Run the mission
execute_mission()
