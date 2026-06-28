from typing import Tuple
from droneresponse_mathtools import Pvector
from scipy.spatial.transform import Rotation, Slerp


Quaternion = Tuple[float, float, float, float]
Vec3 = Tuple[float, float, float]


def interpolate_pos(t_int: float, t0: float, pos0: Vec3, t1: float, pos1: Vec3) -> Vec3:
    """Interpolates a 3D position between two points at two given times. Assumes a constant velocity
    between the two points.

    Positions are given in ECEF coordinates with units of meters.
    Times are given in seconds since epoch.

    Args:
        t_int: time of the desired interpolated position in seconds since epoch
    """
    pos0_ecef = Pvector(*pos0)
    pos1_ecef = Pvector(*pos1)

    ecef_diff = pos1_ecef - pos0_ecef

    frac_of_time_diff = (t_int - t0) / (t1 - t0)

    interpolated_ecef = []
    for i, comp in enumerate(ecef_diff):
        interpolated_ecef.append(comp * frac_of_time_diff + pos0_ecef[i])

    return tuple(interpolated_ecef)


def interpolate_attitude(
    t_int: float,
    t0: float,
    q0: Quaternion,
    t1: float,
    q1: Quaternion
) -> Quaternion:
    """Interpolates an attitude between two attitudes at two given times. Assumes a constant angular
    velocity between the two attitudes.

    Times are given in seconds since epoch.

    Args:
        t_int: time of the desired interpolated attitude in seconds since epoch
    """
    '''
    use SLERP - most accurate
    https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
    Looks straightforward enough to implement explicitly, but scipy already has the capability and
    is an existing dependency
    '''
    rotations = Rotation.from_quat((q0, q1))

    times = (t0, t1)

    slerp = Slerp(times, rotations)

    return tuple(slerp((t_int)).as_quat())

