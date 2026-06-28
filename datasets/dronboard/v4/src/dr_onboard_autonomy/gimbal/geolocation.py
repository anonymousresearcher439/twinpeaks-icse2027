"""Computes the geolocation of objects in the aerial video frame.
"""
from enum import Enum
import math
from dataclasses import dataclass, field
from typing import NamedTuple, Tuple, Union, Optional, Protocol
from typing import Tuple, Type, Union
import numpy as np
from droneresponse_mathtools import Lla, Pvector
from dr_onboard_autonomy.models import DRConfig, Quaternion


import rospy
from tf.transformations import (
    quaternion_matrix,
    quaternion_about_axis,
    quaternion_multiply,
    euler_from_quaternion,
)

QuaternionTuple = Tuple[float, float, float, float] # x, y, z, w


class RayIntersectionError(Exception):
    """Exception raised when a ray does not intersect the plane."""
    def __init__(self, message="Ray does not intersect the plane"):
        self.message = message
        super().__init__(self.message)


def geolocate_object_from_camera(
    fov_h: float,
    lla: Tuple[float, float, float],
    image_res: Tuple[int, int],
    target_coords: Tuple[int, int],
    quaternion_gimbal: QuaternionTuple,
    ground_alt: float=0.0,
    bvh_tree: Optional['BVHTree'] = None
) -> Tuple[float, float, float]:
    """Determines an object's geolocation in WGS84 ellipsoid geodetic coords given information about
    a cameras position, orientation and the location of the target object within its frame

    Parameters:
        fov_h:
            Horizontal field of view of the camera
        lla:
            Geodetic position of the camera relative to the WGS84 ellipsoid
        image_res:
            (horizontal, vertical) resolution in pixels of the camera
        target_coords:
            Raster pixel coordinates (x, y) of the target object within the image frame. The top
            left of the image is 0, 0. x numbered left to right and y numbered top to bottom.
        quaternion_gimbal:
            quaternion in ENU describing the orientation of the camera (x, y, z, w)
        ground_alt:
            altitude above the ellipsoid of the ground at the target (meters)

    Returns:
        LLA of the target object relative to the WGS84 ellipsoid

    World spatial coordinate system reference:
    https://www.mathworks.com/help/map/choose-a-3-d-coordinate-system.html
    """
    camera_ray_projection = CameraRayProjection(
        FOVh=fov_h,
        LLA=lla,
        image_res=image_res,
        target_coors=Coordinates(*target_coords),
        quaternion=quaternion_gimbal,
        ground_alt=ground_alt
    )

    intersect_ECEF = camera_ray_projection.target_location(
        camera_ray_projection.ENU_to_ECEF(camera_ray_projection.target_ENU()),
        bvh_tree
    )

    return(
        camera_ray_projection.ECEFtoLLA(
            intersect_ECEF.x,
            intersect_ECEF.y,
            intersect_ECEF.z
        )
    )


class Coordinates:
    """x, y camera coordinates
    """
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)


class Vector:
    """Contains x,y,z coordinates representing a ray direction or point.
    """
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

    def normalize(self):
        vector = np.array([self.x, self.y, self.z])
        mag = np.linalg.norm(vector)
        norm_vec=vector / mag

        return Vector(norm_vec[0], norm_vec[1], norm_vec[2])


class CameraRayProjection:
    """Finds the target ALL location

    Algorithm:
    -Receive a quaternion in ENU with the drone camera's current orientation
    -Calculate the direction vector/camera ray through a particular pixel in the image taken
    -Transform the camera ray projection to an ENU direction with respect to the drone
    -Transform the camera ENU direction to an ECEF direction vector
    -Define the drone's origin as its position in ECEF
    -Define the ground as the following: (drone's latitude, drones latitude, - (drone's altitude)) then convert it to ECEF
    -Define the normal as (cosλ*cosφ,sinλ*cosφ,sinφ) with  φ = latitude, λ = longitude
    -Take the intersection of the ray to the plane, which is an ECEF point
    -Convert the ECEF point of intersection to LLA
    """
    def __init__(
        self,
        FOVh: float,
        LLA: Tuple,
        image_res: Tuple,
        target_coors: Coordinates,
        quaternion: QuaternionTuple,
        ground_alt: float
    ):
            self.FOVh = FOVh
            self.LLA = LLA
            self.latitude, self.longitude, self.altitude = self.LLA[0], self.LLA[1], self.LLA[2]
            self.image_width, self.image_height = image_res[0], image_res[1]
            self.xy_coors = self._raster_to_xy(target_coors, image_res)
            self.quaternion = quaternion
            self.ground_alt = ground_alt

            #Constant Params
            self.K = self._k_factor()
            self.Alpha = self._alpha_angle()
            self.Beta = self._beta_angle()

    '''
    Generates the parameters used to calculate the camera ray projection through a pixel point.
    '''
    def _k_factor(self):
        return (1 / (2 * np.tan(np.deg2rad(self.FOVh / 2))))

    def _alpha_angle(self):
        x_coors = self.xy_coors[0]
        return (np.arctan(x_coors / (self.image_width * self.K)))

    def _beta_angle(self):
        x_coors, y_coors = self.xy_coors[0], self.xy_coors[1]
        return (np.arctan(y_coors / np.sqrt((self.K * self.image_width) ** 2 + (x_coors) ** 2)))


    def transformation_ENU(self) -> np.array:
        """
        Returns:
            Homogeneous rotation matrix of camera orientation in ENU.
        """
        R = quaternion_matrix(self.quaternion)
        new_array = np.array(R)
        new_array = np.delete(new_array, obj=3, axis=0)
        transformation_matrix = np.delete(new_array, obj=3, axis=1)

        return transformation_matrix


    def transformation_ECEF(self) -> np.array:
        """
        Transformations from:
            - https://gssc.esa.int/navipedia/index.php/Transformations_between_ECEF_and_ENU_coordinates -

        Constants:
            φ  = latitude
            λ = longitude

        Returns:
            Rotation matrix from ENU coordinates to ECEF.
        """
        Phi, Lambda = np.deg2rad(self.latitude), np.deg2rad(self.longitude)

        m00 = -np.sin(Lambda)
        m01 = -np.cos(Lambda) * np.sin(Phi)
        m02 = np.cos(Lambda) * np.cos(Phi)

        m10 = np.cos(Lambda)
        m11 = -np.sin(Lambda) * np.sin(Phi)
        m12 = np.sin(Lambda) * np.cos(Phi)

        m20 = 0
        m21 = np.cos(Phi)
        m22 = np.sin(Phi)

        rotation_matrix = np.array([[m00, m01, m02], [m10, m11, m12], [m20, m21, m22]])

        return rotation_matrix


    def _raster_to_xy(self, raster_coors: Coordinates, image_res: Tuple) -> Tuple[float, float]:
        """
        Params:
            raster_coors:
                Raster pixel coordinates, with the top left of the image as 0,0. Columns numered
                left to right beginning at 0, rows are top to bottom beginning with 0.
            image_res:
                resolution of the image in pixels
        Returns:
            Distance in X and Y in pixels with respect to image center. X ∈ (-w/2,w/2).
        """
        center_x, center_y = image_res[0] / 2, image_res[1] / 2
        x, y = raster_coors.x, raster_coors.y
        new_x = (x - center_x if x >= center_x else - (center_x - x))
        new_y = (center_y - y if y <= center_y else - (y - center_y))
        image_coors = Coordinates(new_x, new_y)
        return (image_coors.x, image_coors.y)


    def pixel_projection(self) -> np.array:
        """
        Return:
            Unit column vector of pixel direction from camera origin.
            [
                [x], -> axis through camera lens center in the direction the camera is pointing
                [y], -> horizontal axis through camera origin orthogonal to x
                [z] -> vertical axis through camera origin orthogonal to x
            ]

        Note: x-axis direction determined by noting the x-axis of the camera coordinate system unit
        vector pointing towards the target is 1 when the target pixels are centered in the raster
        frame
        """
        Vx = np.cos(self.Beta) * np.cos(self.Alpha)
        Vy = -np.cos(self.Beta) * np.sin(self.Alpha)
        Vz = np.sin(self.Beta)
        column_vector = np.array([[Vx], [Vy], [Vz]])

        return column_vector


    def target_ENU(self):
        """Rotate the vector from the camera to target in the camera's local coordinate system
        INTO the ENU coordinate system describing the orientation of the camera relative to the
        WGS84 ellipsoid

        Return:
            Vector from camera to target in ENU
        """
        column_vector = self.pixel_projection()
        rotation_matrix = self.transformation_ENU()
        r1 = rotation_matrix.dot(column_vector)
        V_p = Vector(r1[0, 0], r1[1, 0], r1[2, 0])
        return V_p


    def ENU_to_ECEF(self, target_ENU_vector) -> np.array:
        """
        Params:
            Vector from camera to target in ENU
        Return:
            Vector from camera to target in ECEF
        """
        V_p = np.array([[target_ENU_vector.x], [target_ENU_vector.y], [target_ENU_vector.z]])
        ECEF_rotation_matrix = self.transformation_ECEF()
        r1 = ECEF_rotation_matrix.dot(V_p)
        V_ECEF = Vector(r1[0, 0], r1[1, 0], r1[2, 0])

        return V_ECEF


    @staticmethod
    def LLAtoXYZ (latitude, longitude, altitude) -> Pvector:
        """
        Params:
            Latitude, Longitude, Altitude with height in meters above ellipsoid
        """
        location = Lla(latitude,longitude,altitude)
        pvec = location.to_pvector()
        x,y,z = float(pvec.x), float(pvec.y), float(pvec.z)
        return x,y,z


    @staticmethod
    def ECEFtoLLA(x: float, y: float, z: float) -> Lla:
        """
        Params:
            x, y, z in ECEF (meters)
        """
        location = Pvector(x, y, z)
        lla_pos = location.to_lla()
        latitude, longitude, altitude = lla_pos.latitude, lla_pos.longitude, lla_pos.altitude

        return (latitude, longitude, altitude)



    def target_plane_intersection(self, ray_point, ray_direction):
        Phi, Lambda = np.deg2rad(self.latitude), np.deg2rad(self.longitude)
        ECEF_ground = self.LLAtoXYZ(self.latitude, self.longitude, self.ground_alt)

        plane_point = np.array([ECEF_ground[0], ECEF_ground[1], ECEF_ground[2]])

        #for ground plane, "up" direction
        plane_normal = np.array([
            (np.cos(Lambda) * np.cos(Phi)),
            (np.sin(Lambda) * np.cos(Phi)),
            np.sin(Phi)
        ])

        denom = plane_normal.dot(ray_direction)
        if abs(denom) < 1e-6:
            # in this case the ray is parallel or close to parallel so
            # even if denom is not exactly zero, we can treat it as zero
            # as the ray will extend off an enormous distance before
            # intersecting the plane
            raise RayIntersectionError("The ray is parallel to the plane and does not intersect the plane")
        dir_vector = ray_point - plane_point
        alpha = (-plane_normal.dot(dir_vector)) / denom
        if alpha < 0:
            # in this case the ray is shooting off into space so we need
            # to raise an exception because we can't find the
            # intersection
            raise RayIntersectionError("The ray shoots off into space and does not intersect the plane")
        intersect = dir_vector + (alpha * ray_direction) + plane_point

        return intersect


    def target_location(self, target_ECEF: Vector, bvh_tree: Optional['BVHTree']) -> Vector:
        """Finds the intersection of each of the 4 rays projecting from the camera to the ground.

        Params:
            target_ECEF: direction of target in ECEF
        Return:
            Intersection point ENU

        To change the plane/ground intersection, adjust plane_point.
        The normal for the horizontal plane always points upward at z = 1 in XYZ.
        In ECEF, the normal unit vector is equivalent to (cosλ*cosφ,sinλ*cosφ,sinφ).

        Algorithm from Practical Geometry Algorithms: with C++ Code by Daniel Sunday
        """
        ECEF_drone = self.LLAtoXYZ(self.latitude, self.longitude, self.altitude)
        ECEF_drone = Vector(ECEF_drone[0], ECEF_drone[1], ECEF_drone[2])

        #drone origin ECEF is ray point
        ray_point = np.array([ECEF_drone.x, ECEF_drone.y, ECEF_drone.z])

        #ray_direction is the direction vector to target in ECEF
        ray_direction = np.array([target_ECEF.x, target_ECEF.y, target_ECEF.z])

        # Intersection algorithm using triangle mesh vertices, if available
        if bvh_tree:
            try:
                intersect = bvh_tree.return_intersection_point(ray_point, ray_direction)
            except Exception as e:
                rospy.logerr(f"Error calculating object position using BVH tree: {e}")
                rospy.logwarn("Falling back to plane intersection calculation")
                intersect = self.target_plane_intersection(ray_point, ray_direction)
        else:
            rospy.logwarn("Falling back to plane intersection calculation")
            intersect = self.target_plane_intersection(ray_point, ray_direction)

        return Vector(intersect[0], intersect[1], intersect[2])


class TwoAxisGimbalConfig(Enum):
    ROLL_FIRST = "roll_first"
    PITCH_FIRST = "pitch_first"

    # make a static method that takes a string and returns the corresponding enum
    @staticmethod
    def from_str(s: str):
        s = s.lower()
        if s == "roll_first":
            return TwoAxisGimbalConfig.ROLL_FIRST
        elif s == "pitch_first":
            return TwoAxisGimbalConfig.PITCH_FIRST
        else:
            raise ValueError(f"Invalid string for TwoAxisGimbalConfig: {s}")


@dataclass
class GimbalLimits:
    """The limits of a gimbal in degrees.
    """
    min: float
    max: float

    _rad_min: float = field(init=False)
    _rad_max: float = field(init=False)

    def __post_init__(self):
        self._rad_min = math.radians(self.min)
        self._rad_max = math.radians(self.max)


class GimbalQuatCalculator(Protocol):
    def find_gimbal_quat(self, drone_attitude: Quaternion) -> Tuple[bool, Quaternion]:
        ...


class FixedCameraQuatCalculator(GimbalQuatCalculator):

    @staticmethod
    def build_from_config(config: DRConfig) -> 'FixedCameraQuatCalculator':
        gimbal_config = config.gimbal
        assert gimbal_config.type == "fixed_camera"
        yaw = math.radians(gimbal_config.yaw)
        pitch = math.radians(gimbal_config.pitch)
        roll = math.radians(gimbal_config.roll)
        return FixedCameraQuatCalculator(yaw, pitch, roll)

    def __init__(self, yaw: float, pitch: float, roll: float):
        yaw = quaternion_about_axis(yaw, [0, 0, 1, 0])
        pitch = quaternion_about_axis(pitch, [0, 1, 0, 0])
        roll = quaternion_about_axis(roll, [1, 0, 0, 0])
        fixed_attitude = quaternion_multiply(yaw, quaternion_multiply(pitch, roll))
        self.camera_quat = fixed_attitude

    def find_gimbal_quat(self, drone_attitude: Quaternion) -> Tuple[bool, Quaternion]:
        # TODO implement this...
        # FIXME
        success = False
        return success, quaternion_multiply(drone_attitude, self.camera_quat)


class ThreeAxisGimbalQuatCalculator:

    def __init__(self,
                 roll_limits: Optional[GimbalLimits]=None,
                 pitch_limits: Optional[GimbalLimits]=None,
                 yaw_limits: Optional[GimbalLimits]=None
                ):
        self.roll_limits = roll_limits
        self.pitch_limits = pitch_limits
        self.yaw_limits = yaw_limits

    @staticmethod
    def build_from_config(config: DRConfig) -> 'ThreeAxisGimbalQuatCalculator':
        """Given a DRConfig object, build a ThreeAxisGimbalFrameCalculator.
        """
        axes = config.gimbal.axes
        assert axes == "three_axis"

        roll_limits = None
        if config.gimbal.roll_limits:
            roll_limits = GimbalLimits(
                min=config.gimbal.roll_limits.min,
                max=config.gimbal.roll_limits.max
            )

        pitch_limits = None
        if config.gimbal.pitch_limits:
            pitch_limits = GimbalLimits(
                min=config.gimbal.pitch_limits.min,
                max=config.gimbal.pitch_limits.max
            )

        yaw_limits = None
        if config.gimbal.yaw_limits:
            yaw_limits = GimbalLimits(
                min=config.gimbal.yaw_limits.min,
                max=config.gimbal.yaw_limits.max
            )
        return ThreeAxisGimbalQuatCalculator(
            roll_limits=roll_limits,
            pitch_limits=pitch_limits,
            yaw_limits=yaw_limits
        )

    def _is_limit_ok(self, radians: float, limits: GimbalLimits) -> bool:
        if limits is None:
            return True
        return limits._rad_min <= radians <= limits._rad_max

    def find_gimbal_quat(self, drone_attitude: Quaternion) -> Tuple[bool, Quaternion]:
        # FIXME: THIS DOES NOT WORK
        euler_angles = euler_from_quaternion(drone_attitude)
        roll, pitch, yaw = euler_angles
        roll_ok = self._is_limit_ok(roll, self.roll_limits)
        pitch_ok = self._is_limit_ok(pitch, self.pitch_limits)
        yaw_ok = self._is_limit_ok(yaw, self.yaw_limits)
        success = roll_ok and pitch_ok and yaw_ok
        # should we assume that the gimbal has roll lock, pitch lock, and yaw lock?
        # if so we should return this:
        # return success, Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        # otherwise we should return the drone attitude:
        # FIXME THIS DOES NOT WORK
        # success = False
        return success, drone_attitude


class TwoAxisGimbalQuatCalculator:
    """
    TwoAxisGimbalFrameCalculator calculates the quaternion that rotates from ENU to the gimbal's operating frame (given the drone's attitude).

    This class can do this calculation for a 2-axis gimbal with roll and
    pitch joints when roll lock and pitch lock are enabled. (This class
    does not support two axis gimbals that have a yaw motor.)

    Use this when you want to find the gimbal's operating frame. This
    frame has its forward and right axes level with the horizon. A two
    axis gimbal must respond to the drone's attitude to level the
    forward and right axes. And this takes care of calculating what the
    gimbal must have done to achieve this.

    This class provides a method: `find_gimbal_quat` that calculates the
    gimbal quaternion given the drone's attitude quaternion. Note this
    calculation assumes that "pitch lock" and "roll lock" are enabled.
    For more info see the mavlink v2 specification for these gimbal
    device flags:

    - GIMBAL_DEVICE_FLAGS_PITCH_LOCK: Lock pitch angle to absolute angle
      relative to horizon (not relative to vehicle). This is generally
      the default with a stabilizing gimbal.

    - GIMBAL_DEVICE_FLAGS_ROLL_LOCK: Lock roll angle to absolute angle
      relative to horizon (not relative to vehicle). This is generally
      the default with a stabilizing gimbal.

    https://mavlink.io/en/messages/common.html#GIMBAL_DEVICE_FLAGS

    You initialize this class with parameters that describe the gimbal.
    Then you can call the `find_gimbal_quat` method. The idea is, you'd
    build this object once when the program starts, and then call
    find_gimbal_quat whenever you need to find the gimbal's frame of
    reference using the drone's attitude.

    Important: This finds the rotation that we'd need to apply in ENU to
    reach the gimbal's frame of reference.

    """
    @staticmethod
    def build_from_config(config: DRConfig) -> 'TwoAxisGimbalQuatCalculator':
        """Given a DRConfig object, build a TwoAxisGimbalFrameCalculator object.
        """
        axes = config.gimbal.axes
        assert axes == "roll_first" or axes == "pitch_first"

        roll_limits = None
        if config.gimbal.roll_limits:
            roll_limits = GimbalLimits(
                min=config.gimbal.roll_limits.min,
                max=config.gimbal.roll_limits.max
            )

        pitch_limits = None
        if config.gimbal.pitch_limits:
            pitch_limits = GimbalLimits(
                min=config.gimbal.pitch_limits.min,
                max=config.gimbal.pitch_limits.max
            )

        return TwoAxisGimbalQuatCalculator(
            configuration=axes,
            roll_limits=roll_limits,
            pitch_limits=pitch_limits
        )

    def __init__(self,
        configuration: Union[TwoAxisGimbalConfig, str],
        roll_limits:Optional[GimbalLimits]=None,
        pitch_limits:Optional[GimbalLimits]=None,
    ):
        """
        Initialize an object to calculate the rotation needed to go from
        ENU to the gimbal's frame of reference given a two axis gimbal
        that has pitch lock and roll lock enabled.

        The configuration argument describes how the joints of the
        gimbal are arranged. A two-axis gimbal can be built either roll
        first or pitch first

        - A roll-first gimbal has the roll joint directly connected to
          the drone. The pitch joint is then connected to the roll
          joint.
        - A pitch-first gimbal has the pitch joint directly connected to
          the drone. The roll joint is connected to the pitch joint.

        This configuration argument controls the order that we apply
        rotations when trying to find a frame that's level with the
        horizon. For example, a roll-first gimbal must apply rotation to
        the roll axis before doing any pitch adjustments. So as the
        gimbal tries to level it's front and side axes with the horizon,
        it will first roll (to level the side axes) and then pitch to
        level the forward axis (hence the name roll-first).

        The roll_limits and gimbal_limits are optional arguments but
        highly recommended when working with a real gimbal. Many real
        gimbals have operating limits on the roll and pitch angles. If
        we need to exceed these limits to find a level frame, then we
        will set the "success" flag to False.

        Parameters:
        - configuration (TwoAxisGimbalConfig): The configuration of
            the gimbal, either ROLL_FIRST or PITCH_FIRST. If provided
            as a string, it will be converted to the corresponding
            enum. The string is case insensitive.

        - roll_limits (None or GimbalLimits): The roll limits of the
            gimbal (the min roll, and max roll) in degrees, or None
            for no limits.

        - pitch_limits (None or GimbalLimits):  The pitch limits of
            the gimbal (the min pitch, and max pitch) in degrees, or
            None for no limits.
        """

        if isinstance(configuration, str):
            configuration = TwoAxisGimbalConfig.from_str(configuration)

        self.configuration = configuration
        self.roll_limits = roll_limits
        self.pitch_limits = pitch_limits

    def find_gimbal_quat(self, drone_attitude: Quaternion) -> Tuple[bool, Quaternion]:
        """Given the attitude of the drone, find the rotation needed to reach the gimbal frame in ENU.

        This returns a quaternion that describes the rotation needed to
        reach the gimbal's frame of reference from ENU.

        For context, the gimbal's frame of reference:
        - has it's origin at (or near) the vehicle's position
        - has it's forward and right axes level with the horizon

        This is the frame that gimbal quaternions are expressed in. For
        example, gimbal status messages and gimbal control messages
        specify a quaternion that represents the orientation of the
        camera. This quaternion is expressed in the gimbal frame of
        reference.

        The specific orientation of the x and y axes depends on the
        configuration of the gimbal. For example, if the gimbal has a
        "roll first" configuration (like the two-axis Mio) then we can
        find the frame of reference by:

            1. Roll around the gimbal's x-axis until the gimbal's y-axis
               is level with the horizon
            2. Pitch around the gimbal's y-axis until the gimbal's
               x-axis is level with the horizon

        For a pitch first gimbal we would:

        1. Pitch around the gimbal's y-axis until the gimbal's x-axis is level
        2. Roll around the gimbal's x-axis until the gimbal's y-axis is level

        This method tries to find the chain of rotations needed to bring
        the x and y axes level with the horizon. NOTE: this method is
        not perfect and may not work in all cases. It might a sequence
        of rotations that put gimbal frame upside down. This is because
        this implimentation finds the smallest rotation needed to bring
        the side and forward axes to the horizon. Sometimes the most
        direct way puts the gimbal frame upside down. This is a
        limitation of this approach. As of 2024-09-30, this method is
        used with the Mio Two Axis Gimbal. This gimbal has roll and
        pitch limits and this method finds the correct gimbal frame for
        any situation within the Mio gimbal's operational constraints.
        If the gimbal frame ends up being upside down, the Mio gimbal
        itself won't be able to correct it to a level position.
        Furthermore in case the gimbal frame is upside down, we will set
        the "success" flag to False.

        When the amount of rotation needed exceeds the gimbal's
        operating limits, we will set the "success" flag to False. We
        make no attempt to find a frame that is within the gimbal's
        operating limits. We provide a best effort calculation. We will
        find rotation that levels the forward and side axes with the
        least amount of rotation around the roll and pitch joints.

        Arguments:

        - drone_attitude (Quaternion): The attitude of the drone in ENU.
          This quaternion describes the rotation needed to go from ENU
          to the drone's body frame. This quaternion's coordinate system
          is in ENU.

        Returns:
        A tuple with two elements:
            - A boolean indicating if the calculation was successful.
              This is true when all of the following are true: (1) we
              found rotations that are within the roll and pitch limits
              of the gimbal and (2) the resulting gimbal frame is
              right-side up. (the z-axis aligned with local Earth Up)

            - The quaternion describing how to rotate from ENU to reach
              the gimbal's frame (specified in the ENU coordinate
              system). This is a best effort. We will do the
              theortically correct calculation to bring the forward and
              side axes to the horizon. But there are numerous edge
              cases where this method will not return realistic results.
              For example, if the drone's front axes is pointing nearly
              vertical (like body-forward is 89.99 degrees vertical). In
              this case the amount of rotation needed to bring the side
              axis to the x/y plane could be huge, when realistically
              the gimbal would accept a small amount of error as to not
              overcompensate. Furthermore sometimes this results in a
              gimbal frame that is upside down. This is a limitation of
              this method. If the success flag is False, you should
              probably not trust this quaternion. It might not be
              realistic.
        """
        # TODO create a method for finding the gimbal frame that will
        # never put the gimbal frame upside down

        x = [1, 0, 0, 0]
        y = [0, 1, 0, 0]
        z = [0, 0, 1, 0]

        R_drone = quaternion_matrix(drone_attitude)
        x_drone = np.dot(R_drone, x)
        y_drone = np.dot(R_drone, y)

        if self.configuration == TwoAxisGimbalConfig.ROLL_FIRST:
            axes = [x, y]
            limits = [self.roll_limits, self.pitch_limits]
            first_axis = x_drone
            first_vector = y_drone
        elif self.configuration == TwoAxisGimbalConfig.PITCH_FIRST:
            axes = [y, x]
            limits = [self.pitch_limits, self.roll_limits]
            first_axis = y_drone
            first_vector = x_drone
        else:
            raise ValueError(f"Invalid configuration: {self.configuration}")

        q1_angle = self._find_angle_to_xy_plane(rotation_axis=first_axis, vector=first_vector)
        q1 = quaternion_about_axis(q1_angle, axes[0])

        q_tmp = quaternion_multiply(drone_attitude, q1)
        R_tmp = quaternion_matrix(q_tmp)
        x_tmp = R_tmp.dot(x)
        y_tmp = R_tmp.dot(y)

        if self.configuration == TwoAxisGimbalConfig.ROLL_FIRST:
            second_axis = y_tmp
            second_vector = x_tmp
        elif self.configuration == TwoAxisGimbalConfig.PITCH_FIRST:
            second_axis = x_tmp
            second_vector = y_tmp
        else:
            raise ValueError(f"Invalid configuration: {self.configuration}")

        q2_angle = self._find_angle_to_xy_plane(rotation_axis=second_axis, vector=second_vector)
        q2 = quaternion_about_axis(q2_angle, axes[1])

        # TODO delete this
        # angles = [round(math.degrees(angle), 3) for angle in [q1_angle, q2_angle]]
        # print("find_gimbal_frame  q1_angle, q2_angle = ", angles)

        # check the limits
        is_success = self._is_angle_valid(q1_angle, limits[0])
        is_success = is_success and self._is_angle_valid(q2_angle, limits[1])

        result = quaternion_multiply(q_tmp, q2)
        # check that Up in the gimbal frame aligns with local Earth Up
        R_result = quaternion_matrix(result)
        z_result = R_result.dot(z)
        # we should use the dot product to check if the vectors are aligned
        # if they are perfectly aligned, the dot product should be 1
        # if they are perfectly anti-aligned, the dot product should be -1
        # this can happen in certain edge cases where the most direct
        # way to bring the horizontal axes to the horizon is to put the
        # gimbal frame upside down...
        is_success = is_success and np.dot(z_result, z) > 0.99

        # UPDATE 2024-10-10: Turns out the gimbal's operating frame is Forward-RIGHT-DOWN (not Forward-Left-Up).
        # This means that to reach the gimbal's operating frame, we need to rotate around the +x axis by 180 degrees.
        qx180 = quaternion_about_axis(math.pi, [1, 0, 0, 0])
        result = quaternion_multiply(result, qx180)
        result = Quaternion(x=result[0], y=result[1], z=result[2], w=result[3])
        return is_success, result

    def _find_angle_to_xy_plane(self, rotation_axis, vector) -> float:
        """given a rotation axis and a vector, find the angle to rotate the vector so it's in the xy plane

        - The rotation axis and the vector must be orthogonal.
        - The rotation axis must be a unit vector.

        the result is an angle in radians such that:
            This vector:

            V = Quat(axis=rotation_axis, angle=result).rotate(vector)

            Is in the xy plane (V.z value is 0)

        This Implementation is derived from the Rodrigues Rotation
        Formula:
        https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula

        v_rot = v * cos(theta) + (k cross v) * sin(theta) + k * (k dot v) * (1 - cos(theta))

        However, we know that the dot product of the rotation axis and
        the vector is zero so we can simplify the formula to:

        v_rot = v * cos(theta) + (k cross v) * sin(theta)

        Also we know that the z component of the rotated vector is zero
        so we can simplify further:

        0 = v[2] * cos(theta) + (k cross v)[2] * sin(theta)

        Now we can solve for theta
            -v[2] * cos(theta) = (k cross v)[2] * sin(theta)
            -v[2] * cos(theta) / (k cross v)[2] = sin(theta)
            -v[2] / (k cross v)[2] = sin(theta) / cos(theta)
            -v[2] / (k cross v)[2] = tan(theta)
        Thus:

        theta = atan(-v[2] / (k cross v)[2])

        Args:
            rotation_axis (List[float]): the axis of rotation (unit
            vector with 3 elements or a homogeneous vector). If provided
            as a homogeneous vector, the fourth element is discarded.

            vector (List[float]): the vector to rotate (3 elements
            assumed to be orthogonal to rotation_axis).  If provided as
            a homogeneous vector, the fourth element is discarded.

        """
        # we will accept homogeneous vectors
        if len(rotation_axis) == 4:
            rotation_axis = rotation_axis[:3]
        if len(vector) == 4:
            vector = vector[:3]

        # let's assert that the dot product is zero
        # this means that `vector` is orthogonal to the rotation axis
        assert np.isclose(np.dot(rotation_axis, vector), 0, atol=1e-6)
        # lets assert that we have a unit vector for the rotation axis
        assert np.isclose(np.linalg.norm(rotation_axis), 1, atol=1e-6)

        numerator = -1.0 * vector[2]

        rot_x_vec = np.cross(rotation_axis, vector)
        denominator = rot_x_vec[2]

        return math.atan(numerator/denominator)

    @staticmethod
    def _is_angle_valid(radians: float, limit: GimbalLimits) -> bool:
        if limit is None:
            return True
        return limit._rad_min <= radians <= limit._rad_max

from dr_onboard_autonomy.gimbal.geolocation_boundingvols import BVHTree