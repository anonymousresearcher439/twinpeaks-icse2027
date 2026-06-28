import json
from queue import Empty, SimpleQueue
from threading import Thread
from dr_onboard_autonomy.gimbal.geolocation import RayIntersectionError
from dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components.VisionTrigger import CalculationParameters, Helpers
from .geolocation_boundingvols import init_tree, find_terrain_file
from .geolocation import CameraRayProjection, Coordinates
from typing import Tuple, Union
from pathlib import Path


import rospy

QuaternionTuple = Tuple[float, float, float, float]


class GeoLocator:
    def __init__(self, home_pos: LlaPosition):
        self.work_queue = SimpleQueue()
        self.bvh_tree = None
        self.home_location = home_pos
        self._thread = Thread(target=self._run)

    def load(self, mission_data_directory: Union[str, Path], use_bounding_volumes: bool) -> "GeoLocator":
        mission_data_directory = Path(mission_data_directory)
        if not mission_data_directory.exists() or not any(mission_data_directory.glob("*.json")):
            rospy.logerr(f"GeoLocator: Mission data directory does not exist or contains no JSON files: {mission_data_directory}")
        elif use_bounding_volumes:
            rospy.loginfo(f"GeoLocator: Searching for terrain file")
            terrain_file = find_terrain_file(self.home_location, mission_data_directory)
            rospy.loginfo(f"GeoLocator: Terrain file found: {terrain_file}")
            self.bvh_tree = init_tree(terrain_file, use_cache=True)
        return self


    def start(self, mission_data_directory: Union[str, Path]='/usr/local/share/terrain/', use_bounding_volumes=True):
        self.load(mission_data_directory, use_bounding_volumes)
        self._thread.start()


    def submit_calculations(self, camera_params: CalculationParameters, mqtt_client: MQTTClient):
        job = camera_params, mqtt_client
        self.work_queue.put(job)


    def _run(self):
        while not rospy.is_shutdown():
            try:
                job = self.work_queue.get(timeout=0.5)
            except Empty:
                continue
            camera_params, mqtt_client = job
            self.geolocate(camera_params, mqtt_client)

    """
    def create_BVHtree(self, json_path, pkl_file: Union[Path, None]):
        if pkl_file is not None:
            rospy.loginfo(f"GeoLocator: Loading stored BVH Tree")
            tree = BVHTree.from_joblib(Path(pkl_file))
        else:
            rospy.logerr(f"GeoLocator: Creating BVH Tree from triangle mesh")
            tree = BVHTree.from_json(Path(json_path))
        return tree


    def create_BVHtree(self, filepath, pkl_file: Union[Path, None]):
        print(filepath, pkl_file)
        if pkl_file is not None:
            # Load tree
            rospy.loginfo(f"GeoLocator: Found BVH file: {pkl_file.name}")
            tree = BVHTree.from_joblib(Path(pkl_file))

        else:
            rospy.loginfo(f"GeoLocator: Found terrain JSON file: {filepath.name}.\
                           Loading file and constructing BVH Tree")
            terrain_data = self._get_mission_data(filepath)
            # Request data from terrain service
            mission_grid = MissionGrid(terrain_data)

            # Create triangle objects from triangle mesh
            triangles = mission_grid.return_triangles()
            min_triangles = min(len(triangles) // 20, 500)

            # Get the anchor point in LLA that corresponds to (0,0) in local reference frame
            anchor_lla = mission_grid.anchor_lla
            tree = BVHTree(anchor_lla, mission_grid.bounds, triangles, min_triangles)
        return tree
    """

    def geolocate(self, camera_params: CalculationParameters, mqtt_client: MQTTClient):
        try:
            object_lla_ellipsoid = self.geolocate_object_from_camera(
                fov_h=Helpers.fov_h(camera_params),
                lla=Helpers.lla_tuple(camera_params["drone_position"]),
                image_res=Helpers.image_res(camera_params),
                target_coords=Helpers.target_coords(camera_params),
                quaternion_gimbal=Helpers.to_dr_quat(camera_params["camera_attitude_enu"]),
                ground_alt=camera_params["home_position"]["altitude"],
            )
            is_success = True
            reason = False

        except RayIntersectionError as e:
            is_success = False
            object_lla_ellipsoid = (0, 0, 0)
            rospy.logerr(f"GeoLocator: Error calculating object position: {e}")
            reason = f"Could not calculate the ray intersection position: {e.message}"

        object_lla = LlaPosition(*object_lla_ellipsoid, DATUM_REFERENCE.ELLIPSOID_WGS84).to_amsl()

        output = {
            "timestamp": camera_params["timestamp_ms"],
            "lat": object_lla.latitude,
            "lon": object_lla.longitude,
            "alt_amsl": object_lla.altitude,
            "success": bool(is_success),
        }
        if reason:
            rospy.logwarn(f"GeoLocator: Error calculating object position: {reason}")
            output["reason"] = reason

        res_topic = camera_params["res_topic"]
        # Log the output
        log_msg = f"GeoLocator: Sending geo-location response to the vision service (topic={res_topic}) message:\n```\n"
        log_msg += f"{json.dumps(output, indent=4)}\n"
        log_msg += "```"
        rospy.loginfo(log_msg)
        mqtt_client.publish(res_topic, output)


    def geolocate_object_from_camera(
        self,
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
            vertex_location_path:
                local path to the mission grid numpy array of vertices from triangle mesh

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
            self.bvh_tree
        )

        return(
            camera_ray_projection.ECEFtoLLA(
                intersect_ECEF.x,
                intersect_ECEF.y,
                intersect_ECEF.z
            )
        )


