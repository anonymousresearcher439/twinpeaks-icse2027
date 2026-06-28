import unittest
import numpy as np
from pathlib import Path


class TestGeolocatorBVHTree(unittest.TestCase):
    def test_geolocate_object_from_camera_bvh(self):
        """Test for finding intersection using BVH tree.
        The LLA corresponds to a point at Skyway 36. The BVH tree will be built if there is no
        pickled tree already available.
        Approximate time to run this test:
        With pickled BVH Tree: ~30 seconds
        With JSON file (building tree): ~3 minutes
        """
        from dr_onboard_autonomy.gimbal.geolocator import GeoLocator
        from dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition


        # Load BVH tree
        home_pos = LlaPosition(36.212189, -96.006905, 195, datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84) # skyway 36
        gl = GeoLocator(home_pos)
        gl.load_tree(Path("/usr/local/share/terrain/"), Path("/usr/local/share/terrain/"))

        camera_params = {
            "fov_h": 74,
            "lla": (home_pos.latitude, home_pos.longitude, home_pos.altitude),
            "image_res": (1920, 1080),
            "target_coords": (960, 810),
            "ground_alt": 175
        }

        qs =  [
            (0, 0.7071068, 0, 0.7071068),
            (0.056115267, -0.0154703723, 0.9608545, 0.27086953),
            (0, 0.3826834, 0, 0.9238795),
            (-0.270598, 0.2705981, 0.6532814, 0.6532816),
        ]

        test_cases = [
            {
                "description": "camera looks down and to the west",
                "inputs": {**camera_params, "quaternion_gimbal": qs[0]},
                "target_object_lla": (36.21218899999678, -96.00693278075481, 183.2127296317695)
            },
            {
                "description": "camera looks down (10 degrees?) and to the north west",
                "inputs": {**camera_params, "quaternion_gimbal": qs[1]},
                "target_object_lla": (36.21290054231726, -96.0083389306795, 180.99648120246397)
            },
            {
                "description": "camera looks 45 degrees down and to the east",
                "inputs": {**camera_params, "quaternion_gimbal": qs[2]},
                "target_object_lla": (36.21218899996831, -96.00681791013741, 182.95622261606604)
            },
            {
                "description": "camera looks 45 degrees down and to the north",
                "inputs": {**camera_params, "quaternion_gimbal": qs[3]},
                "target_object_lla": (36.21225721080343, -96.00690499997636, 183.35974127634418)
            }
        ]

        for test in test_cases:
            with self.subTest(msg=test["description"]):
                target_lla = gl.geolocate_object_from_camera(
                    fov_h=test["inputs"]["fov_h"],
                    lla=test["inputs"]["lla"],
                    image_res=test["inputs"]["image_res"],
                    target_coords=test["inputs"]["target_coords"],
                    quaternion_gimbal=test["inputs"]["quaternion_gimbal"],
                    ground_alt=test["inputs"]["ground_alt"]
                )

                self.assertAlmostEqual(target_lla[0], test["target_object_lla"][0], places=5)
                self.assertAlmostEqual(target_lla[1], test["target_object_lla"][1], places=5)
                self.assertAlmostEqual(target_lla[2], test["target_object_lla"][2], places=1)

    def test_init_with_str(self):
        """Test that we can initialize the GeoLocator with the various paths as strings.
        For context the load_tree method accepts the SearchPath type which is defined as:

        DirectoryPath = Union[str, Path]
        SearchPath = Union[DirectoryPath, List[DirectoryPath]]
        """
        from dr_onboard_autonomy.gimbal.geolocator import GeoLocator
        from dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition


        # Load BVH tree
        home_pos = LlaPosition(36.212189, -96.006905, 195, datum_ref=DATUM_REFERENCE.AMSL) # skyway 36
        gl = GeoLocator(home_pos)
        gl.load_tree("/usr/local/share/terrain/", "/usr/local/share/terrain/")
        self.assertIsNotNone(gl.bvh_tree)


class TestTriangle(unittest.TestCase):
    def setUp(self):
        from dr_onboard_autonomy.gimbal.geolocation_boundingvols import Triangle

        # Create triangle object
        vertices_m = np.asarray([
            [0., 0., 216.30212402],
            [5., 0., 216.21620178],
            [5., 4., 216.26379395]
        ])
        vertices_ecef = np.asarray([
            [-539620.81439391, -5124362.63431205, 3746798.44789637],
            [-539620.65249572, -5124359.61468056, 3746802.42779   ],
            [-539615.6791399 , -5124360.08481472, 3746802.58115245]
        ])
        self.triangle = Triangle(vertices_m, vertices_ecef)

    def test_ray_intersects(self):
        # Create ray above triangle
        ray_point = (36.20613975901981, -96.01137431541571, 216.6397368763697) # not used, but for LLA reference
        ray_point_ecef = np.asarray([-539620.65249572, -5124361.61468056, 3746800.42779])
        ray_direction = np.asarray([0.08446606, 0.80243917, -0.5907257])

        test_cases = [
            {
                'description': "Ray origin 0.5m above triangle",
                'ray_point': ray_point_ecef,
                'ray_direction': ray_direction,
                'intersects': True,
                't': 0.3743335401953725,
                'u': 0.4164105118966348,
                'v': 0.02455585660826673,
            }
        ]

        for test in test_cases:
            with self.subTest(msg=test["description"]):
                ray_point_ = test['ray_point']
                ray_direction_ = test['ray_direction']
                intersects, (t, u, v) = self.triangle.ray_intersects(ray_point_, ray_direction_)

                self.assertAlmostEqual(t, test["t"], places=5)
                self.assertAlmostEqual(u, test["u"], places=5)
                self.assertAlmostEqual(v, test["v"], places=5)


class TestBoundingVolumes(unittest.TestCase):
    def setUp(self):
        from dr_onboard_autonomy.gimbal.geolocation_boundingvols import BoundingVolume
        from droneresponse_mathtools import Lla
        self.bv = BoundingVolume(
            anchor_lla=np.array([36.20613975901981, -96.01137431541571, 216.6397368763697]),
            bounds=np.asarray([[0, 100], [0, 200], [210, 260]]),
            triangles=[],
        )

        self.plane_normal_true = np.array([-0.08450301, -0.80245999,  0.59069214])
        self.plane_point_true = np.array([ -539620.09141796, -5124356.28655735,  3746796.50574963])

    def test_MtoECEF(self):
        grid = np.asarray([[0, 0, 210], [0, 10, 225], [10, 0, 221], [10, 10, 228]])
        grid_ecef = self.bv.MtoECEF(grid)

        grid_ecef_true = np.asarray([
            [ -539620.09141796, -5124356.28655735,  3746796.50574963],
            [ -539611.4139382 , -5124369.37071126,  3746805.36612707],
            [ -539620.40234238, -5124359.23916698,  3746811.07233422],
            [ -539611.04883317, -5124365.90363498,  3746815.20718344]
        ])

        # Checking entire array
        np.testing.assert_allclose(grid_ecef, grid_ecef_true, rtol=1e-8)


    def test_cut_volume_in_half(self):
        left_bounds, right_bounds = self.bv.cut_volume_in_half()
        left_true = np.asarray([[0, 100],[0, 100],[210, 260]])
        right_true = np.asarray([[0, 100],[100, 200],[210, 260]])

        np.testing.assert_allclose(left_bounds, left_true)
        np.testing.assert_allclose(right_bounds, right_true)


    def test_create_plane_parameters(self):
        dir_vector = np.array([0, 0, 1])
        corner = np.array([0, 0, 210])
        plane_normal, plane_point = self.bv._create_plane_parameters(corner, dir_vector)

        np.testing.assert_allclose(plane_normal, self.plane_normal_true)
        np.testing.assert_allclose(plane_point, self.plane_point_true)


    def test_target_bounding_plane_intersection(self):
        ray_point_ecef = np.array([ -539569.65947681, -5124354.81172553,  3746853.53165184]) #50,50,238
        ray_direction = np.asarray([0.08446606, 0.80243917, -0.5907257])  # directly down

        intersect = self.bv.target_bounding_plane_intersection(
            self.plane_normal_true, self.plane_point_true, ray_point_ecef, ray_direction
        )

        intersect_true = np.array([-539567.27418838, -5124332.15115787, 3746836.8497896])
        np.testing.assert_allclose(intersect, intersect_true, err_msg="Intersection points are not equal")