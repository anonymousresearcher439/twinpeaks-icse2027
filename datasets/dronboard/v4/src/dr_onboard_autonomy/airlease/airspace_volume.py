from dataclasses import dataclass, field
import math
from typing import Callable, List

from droneresponse_mathtools import Lla, Pvector
import numpy as np

from dr_onboard_autonomy.models.kinematics import LlaPosition, DATUM_REFERENCE
from dr_onboard_autonomy.briar_helpers import BriarLla


@dataclass(frozen=True)
class AirspaceSegment:
    """An air tunnel's start position, end position and radius.

    In typical usage, the drone flies from the start position to the end
    position. The radius is the radius of the tunnel in meters.
    """
    start: LlaPosition = field(metadata={"doc": "The start position of the tunnel."})
    end: LlaPosition = field(metadata={"doc": "The end position of the tunnel."})
    radius: float = field(metadata={"doc": "The radius of the tunnel in meters."})


AirspaceVolume = List[AirspaceSegment]


AirspaceVolumeName = int


TunnelFunc = Callable[[LlaPosition], AirspaceVolume]
TunnelFunc.__doc__ = "A function that takes the current position and returns an AirspaceVolume."


def _to_wgs84_LlaPosition(Lla: Lla) -> LlaPosition:
    """Convert a Lla to a LlaPosition
    The Lla must have its altitude in meters above the ellipsoid
    """
    return LlaPosition(
        latitude=Lla.lat,
        longitude=Lla.lon,
        altitude=Lla.alt,
        datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
    )


# TODO research a better value for this
# this is the radius of the air tunnel
air_tunnel_radius = 5.0


def make_circle_air_tunnel_func(center_pos: LlaPosition) -> TunnelFunc:
    """Generates a function that will provide air tunnel parameters for circular trajectories

    Args:
        center_pos: Alt in meters above the ellipsoid
    """
    center_lla = BriarLla.from_LlaPosition(center_pos).ellipsoid.lla
    def circle_air_tunnel_func(drone_pos: LlaPosition) -> AirspaceVolume:
        current_lla = BriarLla.from_LlaPosition(drone_pos).ellipsoid.lla

        n,e,d = current_lla.distance_ned(center_lla)
        radius = math.sqrt(n**2 + e**2)
        buffer = air_tunnel_radius

        # move horizontally to the center of the circle
        # We do this incase the center position is above or below the drone
        # we want the air lease to be at the same altitude as the drone
        air_tunnel_center = current_lla.move_ned(n, e, 0)
        # drop below the center of the cirle to find the start position
        start_pos = air_tunnel_center.move_ned(0, 0, -buffer)
        # move up from the center of the circle to find the end position
        end_pos = air_tunnel_center.move_ned(0, 0, +buffer)
        return [
            AirspaceSegment(
                _to_wgs84_LlaPosition(start_pos),
                _to_wgs84_LlaPosition(end_pos),
                radius + buffer
            )
        ]

    return circle_air_tunnel_func


def make_waypoint_air_tunnel_func(destination: LlaPosition) -> TunnelFunc:
    """Create a closure (a function with internal state) that returns an AirSpaceVolume from the drone to the destination.

    This returns a TunnelFunc that will create an AirSpaceVolume with one AirspaceSegment that goes from the drone to the destination.

    Args:
        destination: the destination for the drone
    """
    end_pos = BriarLla.from_LlaPosition(destination).ellipsoid.lla

    def find_waypoint_air_tunnel(drone_pos: LlaPosition)-> AirspaceVolume:
        current_pos = BriarLla.from_LlaPosition(drone_pos).ellipsoid.lla
        wp_start_pos = current_pos
        wp_end_pos = end_pos
        return [
            AirspaceSegment(
                _to_wgs84_LlaPosition(wp_start_pos),
                _to_wgs84_LlaPosition(wp_end_pos),
                air_tunnel_radius
            )
        ]

    return find_waypoint_air_tunnel


def make_waypoint_multi_air_tunnel_func(destination: LlaPosition) -> TunnelFunc:
    """Create a closure function that will return an AirSpaceVolume from the drone to the destination with multiple AirspaceSegments.

    Args:
        destination: the end position. This is where the drone is going.
    """
    end_pos = BriarLla.from_LlaPosition(destination).ellipsoid.lla
    '''
    TODO - make the air_tunnel_length smarter - dynamic based on velocity, max accel, and max jerk
    '''
    air_tunnel_length = 8.0
    def waypoint_multi_air_tunnel_func(drone_pos: LlaPosition)-> AirspaceVolume:
        current_pos = BriarLla.from_LlaPosition(drone_pos).ellipsoid.lla
        current_xyz = current_pos.to_pvector()
        end_xyz = end_pos.to_pvector()

        start_to_end = end_xyz.xyz - current_xyz.xyz
        magnitude = np.linalg.norm(start_to_end)
        unit_start_to_end = (
            start_to_end[0] / magnitude,
            start_to_end[1] / magnitude,
            start_to_end[2] / magnitude
        )

        num_tunnels = magnitude / air_tunnel_length
        num_full_length_tunnels = math.floor(num_tunnels)

        tunnels = []
        '''
        create full length tunnels
        '''
        incremental_start_lla = current_pos
        for i in range(num_full_length_tunnels):
            incremental_movement = Pvector(
                *tuple([comp * (air_tunnel_length * (i + 1)) for comp in unit_start_to_end])
            )
            incremental_end_lla = Pvector(*(current_xyz.xyz + incremental_movement.xyz)).to_lla()
            tunnels.append(
                AirspaceSegment(
                    start=_to_wgs84_LlaPosition(incremental_start_lla),
                    end=_to_wgs84_LlaPosition(incremental_end_lla),
                    radius=air_tunnel_radius
                )
            )
            incremental_start_lla = incremental_end_lla

        '''
        case where only single tunnel shorter than full length tunnel
        '''
        if num_full_length_tunnels == 0:
            tunnels.append(
                AirspaceSegment(
                    start=_to_wgs84_LlaPosition(current_pos),
                    end=_to_wgs84_LlaPosition(end_pos),
                    radius=air_tunnel_radius
                )
            )

            return tunnels

        '''
        create final fractional tunnel
        '''
        tunnels.append(
            AirspaceSegment(
                start=_to_wgs84_LlaPosition(incremental_start_lla),
                end=_to_wgs84_LlaPosition(end_pos),
                radius=air_tunnel_radius
            )
        )

        return tunnels

    return waypoint_multi_air_tunnel_func


def make_buffer_air_tunnel_func() -> TunnelFunc:
    """Make an tunnel func for hovering in place
    The air tunnel will start 2.5 meters above the drone and end 2.5 meters below it
    """
    def hover_tunnel_func(drone_pos: LlaPosition) -> AirspaceVolume:
        current_pos = BriarLla.from_LlaPosition(drone_pos).ellipsoid.lla
        buffer = 5.0
        # tunnel with half buffer below and half above current position
        start_pos = current_pos.move_ned(0, 0, -buffer / 2) # go up
        end_pos = current_pos.move_ned(0, 0, +buffer / 2) # go down
        return [
            AirspaceSegment(
                end=_to_wgs84_LlaPosition(start_pos),
                start=_to_wgs84_LlaPosition(end_pos),
                radius=air_tunnel_radius
            )
        ]

    return hover_tunnel_func