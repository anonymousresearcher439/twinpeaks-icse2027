import json
import os
import numpy as np
from droneresponse_mathtools import Lla, Pvector
import open3d as o3d
import typing as t
from typing import List, Type, Union
from dr_onboard_autonomy.gimbal.geolocation import RayIntersectionError
from dr_onboard_autonomy.models.kinematics import LlaPosition
from pathlib import Path
import joblib

import rospy

@np.vectorize
def m_to_lla(anchor, north, east):
    return tuple(anchor.move_ned(north, east, 0))


@np.vectorize
def lla_to_ecef(latitude, longitude, altitude):
    location = Lla(latitude,longitude,altitude)
    pvec = location.to_pvector()
    x,y,z = float(pvec.x), float(pvec.y), float(pvec.z)
    return x, y, z


def ECEFtoLLA(x: float, y: float, z: float) -> Lla:
        """
        Params:
            x, y, z in ECEF (meters)
        """
        location = Pvector(x, y, z)
        lla_pos = location.to_lla()
        latitude, longitude, altitude = lla_pos.latitude, lla_pos.longitude, lla_pos.altitude

        return (latitude, longitude, altitude)


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


def _is_readable_file(file: Path) -> bool:
    return file.is_file() and os.access(file, os.R_OK)


def init_tree(path: Path, use_cache: bool, cache_dirs: List[Path]=[]) -> t.Union["BVHTree", None]:
    if use_cache:
        cached_file_name = f"{path.stem}.joblib.gz"

        for directory in cache_dirs:
            candidate_cached_file = directory / cached_file_name
            if not _is_readable_file(candidate_cached_file):
                continue

            rospy.loginfo(f"GeoLocator: Trying to load cached BVH Tree from {candidate_cached_file}")
            try:
                return BVHTree.from_joblib(candidate_cached_file)
            except Exception as e:
                rospy.logerr(f"GeoLocator: Could not load cached BVH Tree: {e}")
                # well that didn't work, let's keep looking...
                continue

    # if we made it this far, then we couldn't find a cached file
    rospy.loginfo(f"GeoLocator: Building BVH Tree from {path}")
    try:
        bvh_tree = BVHTree.from_json(path)
        if use_cache:
            cache_path = Path(f'~/.cache/dr/{path.stem}.joblib.gz').expanduser()
            cache_path.parent.mkdir(parents=True, exist_ok=True)
            joblib.dump(bvh_tree, cache_path, compress=3, protocol=5)
        return bvh_tree
    except:
        rospy.logerr(f"GeoLocator: Could not load BVH Tree. Defaulting to flat plane calculation")
        return None


def find_terrain_file(home_location: LlaPosition, json_files: List[Path]) -> Union[Path, None]:
    """Locates correct mission file by calculating distances between first point in file and drone location.
    Correct mission file is the one that has it's first coordinate closes to drone home. Allowed buffer: 25km

    Args:
        json_files (List[Path]): List of Paths to JSON files with LLA

    Returns:
        PosixPath: Absolute path to file with LLA
    """
    # Open each file, grab first coordinate, calc distance from drone home location
    best_file = None
    min_distance = 25000 # 25km
    home = Lla(home_location.latitude, home_location.longitude, 0)
    for file in json_files:
        data = json.loads(Path(file).read_text())

        #TODO remove this if statement once we deploy pickled BVH files
        if len(data) > 200000:
            continue

        # Calculate distance from first point to home
        pt = data[0]
        distance = Lla(*pt).distance(home)

        if distance < min_distance:
            min_distance = distance
            best_file = file

    return best_file



class BVHTree:
    """Object to construct and contain the entire bounding volume hierarchy (binary tree)

    - Each node is a BoundingVolume object, which stores all relevant information for calculating
        an intersection between a ray and the volume
    - Each BoundingVolume object has a set of Triangles associated with it

    Algorithm for building BVHTree
    1. Construct triangle mesh and initial bounds for entire mission grid (done with MissionGrid)
    (Note that the bounds are in a local reference frame measured in NED meters from an "anchor point" that acts as the origin)
    2. Instantiate BVHTree with list of all triangle objects in mission grid and bounds of mission grid
    3. root node is created as a BoundingVolume object
    4. Subdivide the bounding volume of the root node into two volumes by halving the longest axis and calculate bounds of each half
    5. Assign the Triangles of the entire mission grid to one of the two halves (details in the BV class)
    6. Create left and right children of the root node, using the subdivided bounds and associated Triangles
    7. Continue this process recursively to build the tree until stopping condition met (min number of triangles per BV)

    Algorithm for searching the BVHTree for an intersection with the camera ray:
    1. Starting with the root node, check for an intersection between the bounding volume and ray
    2. If there is a hit, search the children of the node/volume
    3. Continue searching down the tree until a leaf node has been searched
    4. If a leaf node has a hit, search the triangles in that node/volume
    5. If a leaf node doens't have a hit (should not occur), search through the triangles in the parent volume
    6. Return the intersection point of the ray and triangle
    7. If there is no intersection, return None type and geolocation code will use flat plane approximation

    """
    def __init__(
        self,
        anchor_lla: np.ndarray,
        bounds: np.ndarray,
        triangles: list,
        min_triangles: int
    ):
        self.all_hits = []
        self.leaf_hits = []
        self.min_triangles = min_triangles
        self.anchor_lla = anchor_lla
        self.tree = self.build_tree(bounds, triangles)


    @staticmethod
    def from_joblib(path: Path) -> "BVHTree":
        return joblib.load(path)

    @staticmethod
    def from_json(path: Path) -> "BVHTree":
        mission_grid = MissionGrid(np.asarray(json.loads(path.read_text())))
        return BVHTree._construct_tree_from_grid(mission_grid)

    @staticmethod
    def from_numpy(path: Path) -> "BVHTree":
        mission_grid = MissionGrid(np.load(path))
        return BVHTree._construct_tree_from_grid(mission_grid)

    @staticmethod
    def _construct_tree_from_grid(mission_grid) -> "BVHTree":
        triangles = mission_grid.return_triangles()
        min_triangles = min(len(triangles) // 20, 500)
        return BVHTree(
            mission_grid.anchor_lla,
            mission_grid.bounds,
            triangles,
            min_triangles
        )

    def build_tree(self, bounds: np.ndarray, triangles: list):
        if len(triangles) < self.min_triangles:
            return None

        # Create BV node with geometric bounds and triangles in those bounds
        node = BoundingVolume(self.anchor_lla, bounds, triangles)

        # Create children, including bounds and triangles for each child
        (left_bounds, left_triangles), (right_bounds, right_triangles) = node.create_children()

        # Continue building left and right sides of tree
        node.left = self.build_tree(left_bounds, left_triangles)
        node.right = self.build_tree(right_bounds, right_triangles)
        return node

    def print_tree(self, node, level=0):
        if node is not None:
            self.print_tree(node.right, level + 1)
            print(' ' * 4 * level + '->', len(node.triangles), level)
            self.print_tree(node.left, level + 1)

    def _search_bhv_node(self, node, ray_point_ecef: np.ndarray, ray_direction: np.ndarray):
        if node is None:
            return

        # Get boolean if there is an intersection between bvh volume/node and camera ray
        is_hit = node.ray_intersects(
            ray_point_ecef, ray_direction
        )

        # If a hit in this volume, append to hit list and then search children
        if is_hit:
            if node.left is None and node.right is None:
                self.leaf_hits.append(node)
            self.all_hits.append(node)
            self._search_bhv_node(node.left, ray_point_ecef, ray_direction)
            self._search_bhv_node(node.right, ray_point_ecef, ray_direction)


    def _triangle_intersections(self, triangles: list, ray_point_ecef: np.ndarray, ray_direction: np.ndarray):
        intersections = []
        for tri in triangles:
            intersection, (t, u, v) = tri.ray_intersects(ray_point_ecef, ray_direction)
            if intersection:
                intersections.append(ray_point_ecef + t * ray_direction)
        return intersections


    def return_intersection_point(self, ray_point_ecef: np.ndarray, ray_direction: np.ndarray):
        # Search the tree
        self._search_bhv_node(self.tree, ray_point_ecef, ray_direction)

        # Access triangles in smallest BV that has a hit
        if len(self.leaf_hits) > 0:
            #triangles = self.leaf_hits[-1].triangles
            # Short term fix for oblique camera angles hitting multiple leaf nodes
            triangles = []
            for l in self.leaf_hits:
                triangles += l.triangles
        elif len(self.all_hits) > 0:
            triangles = self.all_hits[-1].triangles
        else:
            raise RayIntersectionError("No BVH intersections found")

        # Search for intersection with all triangles in BV
        intersections = self._triangle_intersections(triangles, ray_point_ecef, ray_direction)
        if len(intersections) == 0:
            raise RayIntersectionError("No triangle intersections found in bounding volume")
        rospy.loginfo(f"{__file__}: Intersection found using BVH")
        return intersections[0]


class Triangle:
    def __init__(self, vertices_m: np.ndarray, vertices_ecef: np.ndarray):
        self.vertices = vertices_m
        self.vertices_ecef = vertices_ecef
        self.extents = self.create_bounding_volume()

    # Create bounding rectangular prism around triangle
    def create_bounding_volume(self):
        xmax, ymax, zmax = self.vertices.max(axis=0)
        xmin, ymin, zmin = self.vertices.min(axis=0)
        self.bounds = np.asarray([[xmin, xmax],[ymin, ymax],[zmin, zmax]])
        self.corners = np.asarray(np.meshgrid(*self.bounds)).T.reshape(-1, 3)

        xextent = xmax - xmin
        yextent = ymax - ymin
        zextent = zmax = zmin
        return (xextent, yextent, zextent)


    def ray_intersects(self, ray_point: np.ndarray, ray_direction: np.ndarray):
        """Moeller-Trumbore intersection algorithm.

        Parameters
        ----------
        ray_start : np.ndarray
            Length three numpy array representing start of point.

        ray_vec : np.ndarray
            Direction of the ray.

        triangle : np.ndarray
            ``3 x 3`` numpy array containing the three vertices of a
            triangle.

        Returns
        -------
        bool
            ``True`` when there is an intersection.

        tuple
            Length three tuple containing the distance ``t``, and the
            intersection in unit triangle ``u``, ``v`` coordinates.  When
            there is no intersection, these values will be:
            ``[np.nan, np.nan, np.nan]``

        """
        # define a null intersection
        null_inter = np.array([np.nan, np.nan, np.nan])

        # break down triangle into the individual points
        v1, v2, v3 = self.vertices_ecef
        eps = 0.000001 # needs to be 1e-10 for lla

        # compute edges
        edge1 = v2 - v1
        edge2 = v3 - v1
        pvec = np.cross(ray_direction, edge2)
        det = edge1.dot(pvec)

        if abs(det) < eps:  # no intersection
            return False, null_inter
        inv_det = 1.0 / det
        tvec = ray_point - v1
        u = tvec.dot(pvec) * inv_det

        if u < 0.0 or u > 1.0:  # if not intersection
            return False, null_inter

        qvec = np.cross(tvec, edge1)
        v = ray_direction.dot(qvec) * inv_det
        if v < 0.0 or u + v > 1.0:  # if not intersection
            return False, null_inter

        t = edge2.dot(qvec) * inv_det
        if t < eps:
            return False, null_inter

        return True, np.array([t, u, v])


class BoundingVolume:
    """Volume object that fully contains a set of triangles

    Algorithm for constructing volume:
    1. Initialize with bounds of volume (in local meter reference frame) and associated triangles in the volume
    2. Also initialize with the anchor_lla point, which is the origin and all other points are in NED distances relative to this point
    3. Calculate plane normals and plane points (in ECEF).

    Algorithm for subdividing volume and assigning triangles:
    1. Calculate longest dimension of volume and cut in half
    2. Create two halves -> "left" and "right"
    3. Loop through all triangles in the current volume and check if any part of a triangle sits within either subvolume
    4. Assign triangle objects to each volume that they sit in (triangles between volumes will be assigned to both)
    Note that the triangle assignment is based on the 8 vertices forming a rectangular prism that fully encapsulates the triangle

    Algorithm for finding if volume is hit:
    1. Calculate intersection point between ray and plane (volume face)
    2. Check if intersection point is inside the plane
    3. Repeat for all volume faces (6 in total)
    4. A hit on any plane is considered a volume hit
    5. If a volume is hit, the subvolumes are then checked for intersections (inside BVHTree object)
    """
    def __init__(self, anchor_lla: np.ndarray, bounds: np.ndarray, triangles: list=[]):
        self.anchor_lla = anchor_lla
        self.bounds = bounds
        self.corners = np.asarray(np.meshgrid(*bounds)).T.reshape(-1,3)
        self.left: BoundingVolume | None = None
        self.right: BoundingVolume | None = None
        self.triangles = triangles
        self.n_triangles = len(triangles)
        self._start_up_tasks()

    def _start_up_tasks(self):
        self.convert_coordinates_to_ecef()
        self._get_plane_normals_and_points()


    def MtoECEF(self, grid):
        anchor_lla = Lla(*self.anchor_lla)
        lats, longs, _ = m_to_lla(anchor_lla, grid[:,0], grid[:,1])
        x, y, z = lla_to_ecef(lats, longs, grid[:,2])
        return np.concatenate([
            x.reshape(-1,1),
            y.reshape(-1,1),
            z.reshape(-1,1)
        ], axis=1)


    def create_children(self):
        left_bounds, right_bounds = self.cut_volume_in_half()
        left_triangles, l_elev_bounds = self._get_triangles_in_volume(left_bounds)
        right_triangles, r_elev_bounds = self._get_triangles_in_volume(right_bounds)

        # Reset vertical bounds to match heights
        left_bounds[2] = l_elev_bounds
        right_bounds[2] = r_elev_bounds
        return (left_bounds, left_triangles), (right_bounds, right_triangles)


    def cut_volume_in_half(self):
        xmax, ymax, zmax = self.bounds.max(axis=1)
        xmin, ymin, zmin = self.bounds.min(axis=1)

        extents = np.asarray((xmax - xmin,ymax - ymin,zmax - zmin))
        idx = extents.argmax()

        # Cut longest axis in half to get new bounds
        half_extent = extents[idx] / 2
        lower_half = [self.bounds[idx].min(), self.bounds[idx].min() + half_extent]
        upper_half = [self.bounds[idx].max() - half_extent, self.bounds[idx].max()]

        left_bounds = self.bounds.copy()
        left_bounds[idx] = lower_half

        right_bounds = self.bounds.copy()
        right_bounds[idx] = upper_half

        return left_bounds, right_bounds


    def convert_coordinates_to_ecef(self):
        # Used for calculating if intersection point is within volume
        self.corners_ecef = self.MtoECEF(self.corners)
        xmax, ymax, zmax = self.corners_ecef.max(axis=0)
        xmin, ymin, zmin = self.corners_ecef.min(axis=0)
        self.bounds_ecef = np.asarray([[xmin, xmax], [ymin, ymax], [zmin, zmax]])


    def _point_in_volume(self, points: np.ndarray, bounds:np.ndarray) -> bool:
        """ Checking if 3 dimensional points are inside the BoundingVolume.
        A single corner that is inside the BV will result in the triangle being "in" the BV
        """
        xbounds, ybounds, zbounds = bounds
        corners_in_volume = np.where(
        (xbounds.min() <= points[:,0]) & (xbounds.max() >= points[:,0]) & \
        (ybounds.min() <= points[:,1]) & (ybounds.max() >= points[:,1]) & \
        (zbounds.min() <= points[:,2]) & (zbounds.max() >= points[:,2])
        )[0]
        return len(corners_in_volume) > 0 # using any() results in false negative if index is 0


    def _get_triangles_in_volume(self, bounds):
        if not self.triangles:
            return []
        triangles_in_volume = []
        min_elev = 9999
        max_elev = -9999

        for triangle in self.triangles:
            if self._point_in_volume(triangle.corners, bounds):
                triangles_in_volume.append(triangle)

                # Get min and max heights for volume
                min_, max_ = triangle.bounds[2]
                min_elev = min(min_elev, min_)
                max_elev = max(max_elev, max_)
        return triangles_in_volume, np.asarray([min_elev, max_elev])


    def _create_plane_parameters(self, corner, dir_vector):
        # Create two points in NED that are used to create the normal
        corner = corner.reshape(-1,3)
        corner_move_1unit = corner + dir_vector

        # Translate into ECEF. Getting 0th index since function is vectorized
        corner_ecef = self.MtoECEF(corner)[0]
        corner_move_1unit_ecef = self.MtoECEF(corner_move_1unit)[0]

        # Create plane normal
        plane_normal = (corner_move_1unit_ecef - corner_ecef)
        plane_normal = Vector(plane_normal[0], plane_normal[1], plane_normal[2]).normalize()
        return np.asarray([plane_normal.x, plane_normal.y, plane_normal.z]), corner_ecef


    def _get_plane_normals_and_points(self):
        # Only need 2 corners of BV to get plane normals and points
        swdown_corner = self.bounds.min(axis=1)
        neup_corner = self.bounds.max(axis=1)

        self.plane_normals = []
        self.plane_points = []

        # swd corner will have normals facing south, west, and down
        for dir_vector in ([[-1,0,0], [0,-1,0], [0,0, 1]]):
            plane_normal, plane_point = self._create_plane_parameters(swdown_corner, dir_vector)
            self.plane_normals.append(plane_normal)
            self.plane_points.append(plane_point)

        # neu corner will have normals facing north, east, and up
        for dir_vector in ([[1,0,0], [0,1,0], [0,0,-1]]):
            plane_normal, plane_point = self._create_plane_parameters(neup_corner, dir_vector)
            self.plane_normals.append(plane_normal)
            self.plane_points.append(plane_point)


    def target_bounding_plane_intersection(
            self,
            plane_normal: np.ndarray,
            plane_point: np.ndarray,
            ray_point: np.ndarray,
            ray_direction: np.ndarray
            ):
        denom = plane_normal.dot(ray_direction)
        dir_vector = ray_point - plane_point
        alpha = (-plane_normal.dot(dir_vector)) / denom
        intersect = dir_vector + (alpha * ray_direction) + plane_point
        return intersect


    def ray_intersects(self, ray_point: np.ndarray, ray_direction: np.ndarray) -> bool:
        hit = False
        for plane_normal, plane_point in zip(self.plane_normals, self.plane_points):
            intersect = self.target_bounding_plane_intersection(plane_normal, plane_point, ray_point, ray_direction)

            if self._point_in_volume(intersect.reshape(-1,3), self.bounds_ecef):
                hit = True
                #print(f"Intersection found: {ECEFtoLLA(*intersect)}")
        return hit



class MissionGrid:
    """Creates a mission grid object that constructs a triangle mesh and triangle objects

    - The data received from the terrain service is sorted first by longitude and then by latitude
    - The anchor point in LLA that is critical for all downstream tasks is the first element of the sorted array
    - The triangle objects will store vertices in both meters (local coordinate system) and ECEF
    """
    def __init__(self, terrain_service_data: np.ndarray):
        self.data = terrain_service_data
        self.LLAtoXYZ = np.vectorize(self._LLAtoXYZ)
        self.create_mission_grid()


    def _LLAtoXYZ(self, latitude, longitude, altitude) -> float:
        """
        Params:
            Latitude, Longitude, Altitude with height in meters above ellipsoid
        """
        location = Lla(latitude,longitude,altitude)
        pvec = location.to_pvector()
        x,y,z = float(pvec.x), float(pvec.y), float(pvec.z)
        return x,y,z

    def convert_lla_to_ecef(self, array: np.ndarray) -> np.ndarray:
        x, y, z = self.LLAtoXYZ(array[:,0], array[:,1], array[:,2])
        return np.concatenate([x.reshape(-1,1), y.reshape(-1,1), z.reshape(-1,1)], axis=1)


    def _transform_grid_to_m(self):
        distances_ned = []
        anchor = Lla(*self.anchor_lla)
        for pt in self.mission_grid_lla:
            distances_ned.append(anchor.distance_ned(Lla(*pt)))

        grid = np.asarray(distances_ned)[:,:2].round(2).astype(int)
        grid = np.concatenate([grid,self.mission_grid_lla[:,2].reshape(-1,1)], axis=1)

        # Get bounds
        xmax, ymax, zmax = grid.max(axis=0)
        xmin, ymin, zmin = grid.min(axis=0)
        bounds = np.asarray([[xmin, xmax], [ymin, ymax], [zmin, zmax]])
        return grid, bounds


    def create_mission_grid(self):
        # Sort terrain service data, first by longitude, then latitude
        data = np.asarray(self.data)
        data = data[data[:,1].argsort()]
        self.mission_grid_lla = data[data[:,0].argsort()]

        # Set anchor point in LLA to allow future transforms from local to LLA to ECEF
        self.anchor_lla = self.mission_grid_lla[0]

        # Transform to local meters coordinate system and get bounds
        self.mission_grid_m, self.bounds = self._transform_grid_to_m()

        # Convert to ECEF
        self.mission_grid_ecef = self.convert_lla_to_ecef(self.mission_grid_lla)


    def _create_point_cloud(self):
        # Create point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(self.mission_grid_m)
        pcd.estimate_normals()
        pcd.orient_normals_to_align_with_direction(np.asarray([0,0,1]))
        return pcd


    def _create_triangle_mesh_o3d(self, pcd: Type[o3d.geometry.PointCloud]) -> Type[o3d.geometry.TriangleMesh]:
        # Create triangle mesh
        pcd.orient_normals_to_align_with_direction(np.asarray([0,0,1]))
        radii = [1, 2, 5, 10, 20]
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pcd, o3d.utility.DoubleVector(radii)
        )
        mesh.compute_vertex_normals()
        print(mesh)
        return mesh


    def _transform_triangle_coords(self, vertices: np.ndarray) -> np.ndarray:
        return np.asarray([
            (tuple(vertices[v[0]]), tuple(vertices[v[1]]), tuple(vertices[v[2]]))
            for v in self.triangle_array
        ])


    def _compute_triangle_vertex_locations(self):
        print("Creating point cloud from mission grid in local meters...")
        pcd = self._create_point_cloud()

        print("Creating triangle mesh from point cloud...")
        mesh = self._create_triangle_mesh_o3d(pcd)

        # Save vertices
        self.triangle_array = np.asarray(mesh.triangles) # array of vertex ids per triangle
        vertices = np.asarray(mesh.vertices) # vertices used to create grid (mission grid)

        # Convert vertices back to ecef from meters
        print("Transforming triangle vertices into LLA and ECEF")
        self.triangle_vertex_locations_m = self._transform_triangle_coords(self.mission_grid_m)
        self.triangle_vertex_locations_lla = self._transform_triangle_coords(self.mission_grid_lla)
        self.triangle_vertex_locations_ecef = self._transform_triangle_coords(self.mission_grid_ecef)


    def return_triangles(self) -> list:
        self._compute_triangle_vertex_locations()
        triangles = []
        for t_m, t_ecef in zip(
            self.triangle_vertex_locations_m, self.triangle_vertex_locations_ecef
        ):
            triangles.append(Triangle(t_m, t_ecef))
        return triangles



