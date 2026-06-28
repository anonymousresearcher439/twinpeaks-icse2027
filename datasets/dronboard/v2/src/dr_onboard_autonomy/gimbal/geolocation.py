"""Computes the geolocation of objects in the aerial video frame.
"""
from typing import Tuple
import numpy as np
from tf.transformations import quaternion_matrix
from droneresponse_mathtools import Lla, Pvector

QuaternionTuple = Tuple[float, float, float, float] 

def geolocate_object_from_camera(
    fov_h: float,
    lla: Tuple[float, float, float],
    image_res: Tuple[int, int],
    target_coords: Tuple[int, int],
    quaternion_gimbal: QuaternionTuple,
    ground_alt: float=0.0
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
        camera_ray_projection.ENU_to_ECEF(camera_ray_projection.target_ENU())
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


    def target_location(self, target_ECEF: Vector) -> Vector:
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
        Phi, Lambda = np.deg2rad(self.latitude), np.deg2rad(self.longitude)
        ECEF_drone = self.LLAtoXYZ(self.latitude, self.longitude, self.altitude)

        ECEF_drone = Vector(ECEF_drone[0], ECEF_drone[1], ECEF_drone[2])

        #drone origin ECEF is ray point 
        ray_point = np.array([ECEF_drone.x, ECEF_drone.y, ECEF_drone.z])
        #ray_direction is the direction vector to target in ECEF 
        ray_direction = np.array([target_ECEF.x, target_ECEF.y, target_ECEF.z])
        # position of ground in ECEF
        ECEF_ground = self.LLAtoXYZ(self.latitude, self.longitude, self.ground_alt)

        plane_point = np.array([ECEF_ground[0], ECEF_ground[1], ECEF_ground[2]])
        
        #for ground plane, "up" direction  
        plane_normal = np.array([
            (np.cos(Lambda) * np.cos(Phi)),
            (np.sin(Lambda) * np.cos(Phi)),
            np.sin(Phi)
        ])

        denom = plane_normal.dot(ray_direction)
        dir_vector = ray_point - plane_point
        alpha = (-plane_normal.dot(dir_vector)) / denom
        intersect = dir_vector + (alpha * ray_direction) + plane_point

        return Vector(intersect[0], intersect[1], intersect[2])