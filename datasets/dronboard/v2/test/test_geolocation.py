import unittest

'''
"FOVh":80,
            "LLA":[41.687970, -86.249996, 100],
            "Image Res":[1080,200],
            "Target Coors":[350,200],"Quaternion":[0,0,0,0]
'''
class TestGeolocation(unittest.TestCase):
    def test_geolocate_object_from_camera(self):
        from dr_onboard_autonomy.gimbal.geolocation import geolocate_object_from_camera

        '''
        target_object_lla calculated using droneresponse_mathtools Lla.move_ned
        quaternion calculated with:
        https://www.andre-gaschler.com/rotationconverter/
        quaternion rotation validated with:
        https://www.wolframalpha.com/input?i=draw+0.9238795%2B0i-0.3826834j%2B0k+as+a+rotation+operator
        '''
        test_cases = [
            {   
                "description": "camera looks 45 degrees down and to the east with object centered",
                "inputs": {
                    "fov_h": 80,
                    "lla": (0.0, 0.0, 50.0),
                    "image_res": (200, 200),
                    "target_coords": (100, 100), 
                    "quaternion_gimbal": (0, 0.3826834, 0, 0.9238795),
                    "ground_alt": 0.0
                },
                "target_object_lla": (0.0, 0.0004491576420505598, 0.0)
            },
            {   
                "description": "camera looks 45 degrees down and to the north with object centered",
                "inputs": {
                    "fov_h": 80,
                    "lla": (0.0, 0.0, 50.0),
                    "image_res": (200, 200),
                    "target_coords": (100, 100), 
                    "quaternion_gimbal": (-0.270598, 0.2705981, 0.6532814, 0.6532816),
                    "ground_alt": 0.0
                },
                "target_object_lla": (0.0004521847391196666, 0.0, 0.0)
            },
            {   
                "description": "camera looks 45 degrees down and to the east with object right",
                "inputs": {
                    "fov_h": 80,
                    "lla": (0.0, 0.0, 50.0),
                    "image_res": (200, 200),
                    "target_coords": (150, 100),
                    "quaternion_gimbal": (0, 0.3826834, 0, 0.9238795),
                    "ground_alt": 0.0
                },
                "target_object_lla": (-0.0002682965799961742, 0.0004491576420505598, 0.0)
            },
            {   
                "description": "camera looks 45 degrees down and to the east with object right and non-zero ground",
                "inputs": {
                    "fov_h": 80,
                    "lla": (0.0, 0.0, 50.0),
                    "image_res": (200, 200),
                    "target_coords": (150, 100),
                    "quaternion_gimbal": (0, 0.3826834, 0, 0.9238795),
                    "ground_alt": 20.0
                },
                "target_object_lla": (-0.0001609777671245695, 0.0002694937401791448, 20.0000955583)
            }
        ]

        for test in test_cases:
            with self.subTest(msg=test["description"]):
                target_lla = geolocate_object_from_camera(
                    fov_h=test["inputs"]["fov_h"],
                    lla=test["inputs"]["lla"],
                    image_res=test["inputs"]["image_res"],
                    target_coords=test["inputs"]["target_coords"],
                    quaternion_gimbal=test["inputs"]["quaternion_gimbal"],
                    ground_alt=test["inputs"]["ground_alt"]
                )

                self.assertAlmostEqual(target_lla[0], test["target_object_lla"][0])
                self.assertAlmostEqual(target_lla[1], test["target_object_lla"][1])
                self.assertAlmostEqual(target_lla[2], test["target_object_lla"][2], 3)