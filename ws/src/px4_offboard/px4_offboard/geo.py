"""Latitude/longitude to PX4 local NED, using PX4's own map projection."""

import math

# PX4's CONSTANTS_RADIUS_OF_EARTH (and px4_ros2's kRadiusOfEarth) [m]
RADIUS_OF_EARTH = 6371000.0


def global_to_local(lat, lon, ref_lat, ref_lon):
    """Return (north, east) [m] of lat/lon [deg] relative to the reference ref_lat/ref_lon [deg].

    Azimuthal equidistant projection, a port of PX4's MapProjection::project() (also
    px4_ros2's MapProjectionImpl::globalToLocal()), so with the reference set to PX4's local
    origin (vehicle_local_position ref_lat/ref_lon) the result is in PX4's local frame.
    """
    lat_rad, lon_rad = math.radians(lat), math.radians(lon)
    ref_lat_rad, ref_lon_rad = math.radians(ref_lat), math.radians(ref_lon)
    sin_lat, cos_lat = math.sin(lat_rad), math.cos(lat_rad)
    cos_d_lon = math.cos(lon_rad - ref_lon_rad)

    arg = math.sin(ref_lat_rad) * sin_lat + math.cos(ref_lat_rad) * cos_lat * cos_d_lon
    c = math.acos(min(max(arg, -1.0), 1.0))
    k = c / math.sin(c) if abs(c) > 0 else 1.0

    north = k * (math.cos(ref_lat_rad) * sin_lat
                 - math.sin(ref_lat_rad) * cos_lat * cos_d_lon) * RADIUS_OF_EARTH
    east = k * cos_lat * math.sin(lon_rad - ref_lon_rad) * RADIUS_OF_EARTH
    return north, east
