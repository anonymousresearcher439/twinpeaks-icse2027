from pathlib import Path
import unittest
import json
from typing import List
from unittest.mock import MagicMock, NonCallableMock, call, Mock, NonCallableMagicMock, patch

from dr_onboard_autonomy.common import loaders
from dr_onboard_autonomy.models.config import CameraConfig, DRConfig
from dr_onboard_autonomy.mqtt_client import MQTTClient
import smach

from dr_onboard_autonomy.mission_helper import (
    process_altitudes,
    MissionBuilder,
)
from dr_onboard_autonomy.message_senders import ReusableMessageSenders
from dr_onboard_autonomy.models import GimbalAxes
from dr_onboard_autonomy.briar_helpers import BriarLla
from test_UpdateStarePosition import mock_message_senders
from .mock_types import (
    mock_base_state,
    mock_copter_drone,
    mock_drone_config,
    mock_mqtt_message,
    mock_state_kwargs,
    mock_mission_builder_args,
    mock_mqtt_client,
    mock_state_factory,
    read_mission_file,
    find_config_file_path,
    mock_state_factory2,
)

MockMQTTClient = type('MockMQTTClient', (MagicMock, MQTTClient), {})


class TestMissionBuilder(unittest.TestCase):

    def build_mission_builder(self, **kwargs):
        mock_args = mock_mission_builder_args()
        mock_args.update(kwargs)
        builder = MissionBuilder(
            **mock_args
        )
        return builder

    # a test method needs to start with the string "test"
    def test_init(self):
        mock_args = mock_state_kwargs()

        with patch("dr_onboard_autonomy.mission_helper.TwoAxisGimbalQuatCalculator") as MockGimbalQuatCalc:
            mockGimbalQuatInstance = NonCallableMock()
            MockGimbalQuatCalc.build_from_config.return_value = mockGimbalQuatInstance
            builder = MissionBuilder(
                state_factory=mock_state_factory(),
                uav_id="test_drone",
                mqtt_client=mock_args["mqtt_client"],
                local_mqtt_client=mock_args["local_mqtt_client"],
                reusable_message_senders=mock_args["reusable_message_senders"],
                drone=mock_args["drone"],
                air_lease_protocol_fsm=mock_args["air_lease_protocol_fsm"],
                drone_config=mock_drone_config()
            )

        self.assertIsInstance(builder, MissionBuilder)
        builder._setup_tasks.clear()
        output_args = builder.build_kwargs({})
        expected_args = {
            "air_lease_protocol_fsm": mock_args["air_lease_protocol_fsm"], 
            "drone": mock_args["drone"],
            "mqtt_client": mock_args["mqtt_client"], 
            "local_mqtt_client": mock_args["local_mqtt_client"],
            "mission_builder": builder, 
            "reusable_message_senders": mock_args["reusable_message_senders"],
            "uav_id": "test_drone",
            "data": {},
            "heartbeat_handler": False,
            "home": None,
            "home_altitude_offset": None,
            "performance_analysis": False,
            "gimbal_calculator": mockGimbalQuatInstance,
            "camera_config": CameraConfig(horizontal_fov=74.0, vertical_fov=42.0),
        }
        self.maxDiff = None
        self.assertDictEqual(output_args, expected_args)
    
    def test_init_with_mock_args(self):
        """Using mock args, build a MissionBuilder and check it initializes correctly
        """
        builder = self.build_mission_builder()
        self.assertIsInstance(builder, MissionBuilder)

        # make sure there are not setup tasks since we gave mock args for everything
        self.assertListEqual(builder._setup_tasks, [])
        builder.setup()
        self.assertTrue(builder._is_setup_done)

        output = builder.build_kwargs({})
        self.assertEqual(output['mqtt_client'].broker_address, "mqtt")
        self.assertEqual(output['local_mqtt_client'].broker_address, "127.0.0.1")
        self.assertIsInstance(output['reusable_message_senders'], ReusableMessageSenders)
        self.assertEqual(output['uav_id'], "test_drone")
        self.assertEqual(output['mission_builder'], builder)


    @patch("dr_onboard_autonomy.mission_helper.NewGimbalManager")
    @patch("dr_onboard_autonomy.mission_helper.ArdupilotCopterMavrosDrone")
    @unittest.skip("TODO FIX THIS UNIT TEST")
    def test_init_drone_config_ardupilot_and_gimbal_manager(
            self,
            mock_ardupilot_drone: NonCallableMagicMock,
            mock_gimbal_manager: NonCallableMagicMock
        ):
        drone_config = mock_drone_config(gimbal_type="mavlink_gimbal_manager")
        builder = self.build_mission_builder(drone=None, drone_config=drone_config)

        mock_ardupilot_drone.assert_called_once()
        mock_gimbal_manager.assert_called_once()

        gimbal_manager_kwargs: dict = mock_gimbal_manager.call_args.kwargs

        self.assertEqual(
            gimbal_manager_kwargs["gimbal_axes"],
            GimbalAxes.PITCH | GimbalAxes.ROLL
        )

        # TODO: assert ardupilot takeoff state registration


    @patch("dr_onboard_autonomy.mission_helper.GimbalGremsy")
    @patch("dr_onboard_autonomy.mission_helper.ArdupilotCopterMavrosDrone")
    def test_init_drone_config_ardupilot_and_gimbal_gremsy(
            self,
            mock_ardupilot_drone: NonCallableMagicMock,
            mock_gimbal_gremsy: NonCallableMagicMock
        ):
        drone_config = mock_drone_config(gimbal_type="gremsy", gimbal_yaw=True)
        builder = self.build_mission_builder(drone=None, drone_config=drone_config)
        mock_ardupilot_drone.assert_called_once()
        mock_gimbal_gremsy.assert_called_once()

        gimbal_manager_kwargs: dict = mock_gimbal_gremsy.call_args.kwargs

        self.assertEqual(
            gimbal_manager_kwargs["gimbal_axes"],
            GimbalAxes.PITCH | GimbalAxes.ROLL | GimbalAxes.YAW
        )

    @unittest.skip("TODO FIX THIS UNIT TEST")
    @patch("dr_onboard_autonomy.mission_helper.ArdupilotCopterMavrosDrone")
    @patch("dr_onboard_autonomy.mission_helper.GimbalGremsy")
    @patch("dr_onboard_autonomy.mission_helper.MQTTClient", new=MockMQTTClient)
    def test_init_drone_config_ardupilot_and_gimbal_gremsy2(
        self,
        # mock_MQTTClient: MagicMock,
        mock_GimbalGremsy: MagicMock,
        mock_ArdupilotCopterMavrosDrone: MagicMock,
    ):
        MockMQTTClient.return_value = mock_mqtt_client()
        # mock_MQTTClient.return_value = mock_mqtt_client()
        mock_GimbalGremsy.return_value = NonCallableMock()
        mock_ArdupilotCopterMavrosDrone.return_value = mock_copter_drone()
        config_json = "test_configs/test_ardu_mio.toml"
        config_json = "test_configs/matt_lab.toml"
        config_path = find_config_file_path(config_json)
        config = loaders.load_model(DRConfig, config_path, {})
        self.assertEqual(config.name, "matt_lab")
        self.assertEqual(config.system_id, 1)
        self.assertEqual(config.mqtt.host, "192.168.0.7")
        self.assertEqual(config.mqtt_local.host, "mqtt_local")
        self.assertEqual(config.performance_analysis, False)
            
        mission_builder = MissionBuilder(
            uav_id=config.name,
            mqtt_client=config.mqtt.host,
            local_mqtt_client=config.mqtt_local.host,
            performance_analysis=config.performance_analysis,
            system_id=config.system_id,
            drone_config=config,
            home=BriarLla.from_args(41.6066954, -86.3555018, 230.0, is_amsl=True),
            reusable_message_senders=mock_message_senders(),
        )
        state_args = mission_builder.build_kwargs({})

        gimbal_calc = state_args.get("gimbal_calculator")
        self.assertIsNotNone(gimbal_calc)
        from dr_onboard_autonomy.gimbal.geolocation import ThreeAxisGimbalQuatCalculator, TwoAxisGimbalQuatCalculator, FixedCameraQuatCalculator

        self.assertIsInstance(gimbal_calc, TwoAxisGimbalQuatCalculator)


    @patch("dr_onboard_autonomy.mission_helper.NewGimbalManager")
    @patch("dr_onboard_autonomy.mission_helper.Px4CopterMavrosDrone")
    def test_init_drone_config_px4_and_gimbal_manager(
            self,
            mock_px4_drone: NonCallableMagicMock,
            mock_gimbal_manager: NonCallableMagicMock
        ):
        drone_config = mock_drone_config(fcu_type="px4", gimbal_type="mavlink_gimbal_manager")
        builder = self.build_mission_builder(drone=None, drone_config=drone_config)

        mock_px4_drone.assert_called_once()
        mock_gimbal_manager.assert_called_once()

        gimbal_manager_kwargs: dict = mock_gimbal_manager.call_args.kwargs

        self.assertEqual(
            gimbal_manager_kwargs["gimbal_axes"],
            GimbalAxes.PITCH | GimbalAxes.ROLL
        )

        # TODO: assert px4 takeoff state registration


    @patch("dr_onboard_autonomy.mission_helper.GimbalGremsy")
    @patch("dr_onboard_autonomy.mission_helper.Px4CopterMavrosDrone")
    def test_init_drone_config_px4_and_gimbal_gremsy(
            self,
            mock_px4_drone: NonCallableMagicMock,
            mock_gimbal_gremsy: NonCallableMagicMock
        ):
        drone_config = mock_drone_config(fcu_type="px4", gimbal_type="gremsy", gimbal_yaw=True)
        builder = self.build_mission_builder(drone=None, drone_config=drone_config)
        mock_px4_drone.assert_called_once()
        mock_gimbal_gremsy.assert_called_once()

        gimbal_manager_kwargs: dict = mock_gimbal_gremsy.call_args.kwargs

        self.assertEqual(
            gimbal_manager_kwargs["gimbal_axes"],
            GimbalAxes.PITCH | GimbalAxes.ROLL | GimbalAxes.YAW
        )


    @unittest.skip("TODO FIX THIS UNIT TEST")
    @patch("dr_onboard_autonomy.mission_helper.MQTTClient", new=MockMQTTClient)
    def test_mqtt_init(self):
        """
        Test the initialization of a MissionBuilder with MQTT arguments specified as host strings.
        The MissionBuilder is expected to initialize the MQTTClient objects and connect.
        """
        
        def make_mock_mqtt_client(uav_name, broker_address):
            mqtt = mock_mqtt_client()
            mqtt.uav_name = uav_name
            mqtt.broker_address = broker_address
            return mqtt

        MockMQTTClient.side_effect = make_mock_mqtt_client

        mock_state_factory = Mock()
        mock_state_factory.side_effect = mock_base_state

        mock_args = mock_state_kwargs()

        # Initialize MissionBuilder with mock arguments
        builder = MissionBuilder(
            state_factory=mock_state_factory,
            uav_id="test_drone",
            mqtt_client="mqtt",
            local_mqtt_client="mqtt_local",
            reusable_message_senders=mock_args["reusable_message_senders"],
            drone=mock_args["drone"],
            drone_config=mock_drone_config(),
            air_lease_protocol_fsm=mock_args["air_lease_protocol_fsm"],
        )

        self.assertIsInstance(builder, MissionBuilder)

        output_args = builder._build_kwargs_internal({})

         # Check if the MQTTClient objects are initialized and connected correctly
        self.assertEqual(output_args["mqtt_client"].broker_address, "mqtt")
        self.assertEqual(output_args["mqtt_client"].uav_name, "test_drone")
        output_args["mqtt_client"].connect.assert_called_once()

        self.assertEqual(output_args["local_mqtt_client"].broker_address, "mqtt_local")
        self.assertEqual(output_args["local_mqtt_client"].uav_name, "test_drone")
        output_args["local_mqtt_client"].connect.assert_called_once()

        # Assert that the MQTTClient objects are initialized, without checking the order of initialization
        # This is because the order of initialization is not important in this context, only that initialization occurred
        MQTTClient.assert_has_calls([
            call("test_drone", "mqtt"),
            call("test_drone", "mqtt_local"),
        ], any_order=True)
    
    def test_build_mission(self):
        builder = self.build_mission_builder()
        builder.setup()
        mission_json = read_mission_file("test_transform_relative_altitude_no_relative_alts_found.json")
        mission = builder.build_mission(mission_json)
        self.assertIsInstance(mission, smach.StateMachine)
    
    def test_build_task(self):
        builder = self.build_mission_builder()
        builder.setup()
        mission_json = read_mission_file("test_task.json")
        mission, extra_outcomes = builder.build_task(mission_json)
        mission.check_consistency()
        self.assertIsInstance(mission, smach.StateMachine)
        self.assertIsInstance(extra_outcomes, list)
    
    def test_build_task2(self):
        """This unit test was created from simulation data. WE found a bug when running a simulation. This test captures the MQTT message that caused the bug and tests the relevant code.
        """
        task_str = """
         {"task_id": "231bd8b8-594c-4315-95be-caa7ed3d0142", "states": [{"name": "PhasedCircle", "class": "PhasedCircle", "args": {"pitch": 0, "distance": 15, "starting_angle": 149.37827322779478, "center_position": {"latitude": 41.86510686948643, "longitude": -85.90944804402487, "relative_altitude": 20.00415244934392}, "stare_position": {"latitude": 41.86510686948643, "longitude": -85.90944804402487, "relative_altitude": 0}, "total_sweep_angle": 1080, "speed": 6.0, "cruising_altitude": 20.00415244934392, "number_arcs": 1, "pause_time": 1.0}, "transitions": [{"target": "success", "condition": "succeeded_circle"}]}]}
         """
        builder = self.build_mission_builder()
        builder.setup()
        mission, extra_outcomes = builder.build_task(task_str)
        mission.check_consistency()
        task_cancel_payload_wrong = """
        {"uavid": "lime", "task_id": "efe6e363-02cf-4b7f-b95e-a04dd7e6f35d"}
        """
        task_cancel_wrong_msg = mock_mqtt_message("cancel_current_task", "drone/lime/task/cancel-current", task_cancel_payload_wrong)
        task_cancel_payload_correct = """
        {"uavid": "lime", "task_id": "231bd8b8-594c-4315-95be-caa7ed3d0142"}
        """
        task_cancel_correct_msg = mock_mqtt_message("cancel_current_task", "drone/lime/task/cancel-current", task_cancel_payload_correct)
        for name, state in mission.get_children().items():
            print(state)
            self.assertEqual(state.task_id, "231bd8b8-594c-4315-95be-caa7ed3d0142")
            self.assertEqual(state.task_canceled_trigger.task_id, "231bd8b8-594c-4315-95be-caa7ed3d0142")
            self.assertEqual(state.handlers.notify(task_cancel_correct_msg), "task_canceled")
            self.assertIsNone(state.handlers.notify(task_cancel_wrong_msg))
    
    def test_flywaypoints_alts(self):
        """Test that we preprocess the altitude data for FlyWaypoints correctly
        
        This test is in response to issue #434
        https://github.com/DroneResponse/DR-OnboardAutonomy/issues/434

        We will try to parse/build a mission with multiple FlyWaypoints states.
        In our mission we provide the altitude data in every possible way for FlyWaypoints.
        Then we will make sure the the FlyWaypoints state is created with the correct altitude
        every time.

        The FlyWaypoints state is a special case for altitude data. No matter
        how you provide it's altitude, it is interpreted as a relative altitude.

        So you could provide waypoints with `altitude` or `relative_altitude` properties 
        and mission_helper will interpret it as a relative altitude in both cases.

        When MissionHelper goes to init a FlyWaypoints state, it will calculate
        the altitude AMSL for each waypoint (by adding the home altitude to the
        altitude value provided in the mission JSON).

        These calculated altitudes (AMSL) are then used to create the FlyWaypoints state.
        """
        from dr_onboard_autonomy.states import FlyWaypoints
        HOME_ALT = 230.0
        def check_waypoints(expected_waypoints: List[BriarLla], actual_waypoints: List[BriarLla], comment: str = ""):
            self.assertEqual(len(expected_waypoints), len(actual_waypoints))
            for expected, actual in zip(expected_waypoints, actual_waypoints):
                expected = expected.amsl.lla
                actual = actual.amsl.lla
                self.assertAlmostEqual(expected.latitude, actual.latitude, msg=comment)
                self.assertAlmostEqual(expected.longitude, actual.longitude, msg=comment)
                self.assertAlmostEqual(expected.altitude, actual.altitude, msg=comment)
        
        builder = self.build_mission_builder(
            state_factory=mock_state_factory2({
                "FlyWaypoints": FlyWaypoints
            }),
            home=BriarLla.from_args(41.6066954, -86.3555018, HOME_ALT, is_amsl=True),
        )
        builder.setup()
        mission_json = read_mission_file("test_flywaypoints_alts.json")
        mission = builder.build_mission(mission_json)

        # make sure the mission state machine is built correctly
        # and consistent
        self.assertIsInstance(mission, smach.StateMachine)
        mission.check_consistency()

        # now we will check the FlyWaypoints states
        children = mission.get_children()
        assert "FlyWaypoints-1" in children
        # our mission includes 3 FlyWaypoints states
        fw1 = children.get("FlyWaypoints-1")
        fw2 = children.get("FlyWaypoints-2")
        fw3 = children.get("FlyWaypoints-3")
        self.assertIsInstance(fw1, FlyWaypoints)
        self.assertIsInstance(fw2, FlyWaypoints)
        self.assertIsInstance(fw3, FlyWaypoints)

        # check the waypoints for each FlyWaypoints state
        # fw1 has a single waypoint with altitude 10
        expected_pos1 = BriarLla.from_dict({
            "latitude": 41.6086606438366,
            "longitude": -86.35817639529705,
            "altitude": 10 + HOME_ALT
        })
        expected_pos1 = [expected_pos1]
        actual_pos1 = [wp.position for wp in fw1.waypoints]
        check_waypoints(expected_pos1, actual_pos1, """Make sure FlyWaypoints-1 has the correct waypoint altitude. It's one and only waypoint was provided as a dictionary with an altitude property.""")

        # fw2 has a single waypoint with relative_altitude 20
        expected_pos2 = BriarLla.from_dict({
            "latitude": 41.6086606438366,
            "longitude": -86.35817639529705,
            "altitude": 20 + HOME_ALT
        })
        expected_pos2 = [expected_pos2]
        actual_pos2 = [wp.position for wp in fw2.waypoints]
        check_waypoints(expected_pos2, actual_pos2, """Make sure FlyWaypoints-2 has the correct waypoint altitude. It's one and only waypoint was provided as a dictionary with a relative_altitude property.""")

        # fw3 has two waypoints:
        # 1) using altitude = 30
        # 2) using relative_altitude = 40
        expected_pos3 = [
            BriarLla.from_dict({
                "latitude": 41.6086606438366,
                "longitude": -86.35817639529705,
                "altitude": 30 + HOME_ALT
            }),
            BriarLla.from_dict({
                "latitude": 41.6086606438366,
                "longitude": -86.35817639529705,
                "altitude": 40 + HOME_ALT
            }),
        ]
        actual_pos3 = [wp.position for wp in fw3.waypoints]
        check_waypoints(expected_pos3, actual_pos3, """Make sure FlyWaypoints-3 has the correct waypoint altitudes. It was provided a list with two waypoints, one with an altitude property and the other with a relative_altitude property.""")
    


class TestTransformRelativeAltitude(unittest.TestCase):
    def test_transform_altitude_with_flywaypoint(self):
        """Test that altitude data for FlyWaypoint is transformed correctly

        The FlyWaypoint state's `altitude` arg is interpreted with special-case
        logic. This tests that the logic is working correctly.
        """
        take_off_mission = read_mission_file("test_transform_relative_altitude_with_flywaypoint1.json")
        mission = json.loads(take_off_mission)
        altitude = 100
        transformed_mission = process_altitudes(mission, altitude)
        expected_mission = read_mission_file("test_transform_relative_altitude_with_flywaypoint2.json")
        expected_mission = json.loads(expected_mission)
        self.assertDictEqual(expected_mission, transformed_mission)
    
    def test_transform_relative_altitude_no_relative_alts_found(self):
        """Test that the mission is not changed if there are no relative altitudes
        """
        take_off_mission = read_mission_file("test_transform_relative_altitude_no_relative_alts_found.json")
        mission = json.loads(take_off_mission)
        altitude = 0
        transformed_mission = process_altitudes(mission, altitude)
        self.assertDictEqual(mission, transformed_mission)
    
    def test_transform_relative_altitude_no_relative_alts_and_no_takeoff_args(self):
        take_off_mission = read_mission_file("test_transform_relative_altitude_no_relative_alts_and_no_takeoff_args.json")
        mission = json.loads(take_off_mission)
        altitude = 0
        transformed_mission = process_altitudes(mission, altitude)
        self.assertDictEqual(mission, transformed_mission)

    def test_transform_relative_altitude_no_relative_alts_and_takeoff_alt(self):
        take_off_mission = read_mission_file("test_transform_relative_altitude_no_relative_alts_and_takeoff_alt.json")
        mission = json.loads(take_off_mission)
        altitude = 0
        transformed_mission = process_altitudes(mission, altitude)
        self.assertDictEqual(mission, transformed_mission)

    def test_transform_relative_altitude_with_relative_alt_in_takeoff_state(self):
        """Test that the relative altitude in the Takeoff state is transformed

        The Takeoff state is a special case, because it has a `altitude` arg that
        is already interpreted as a relative altitude. So we need to replace
        property without adding the home altitude to it.
        """
        take_off_mission = read_mission_file("test_transform_relative_altitude_with_relative_alt_in_takeoff_state.json")
        mission = json.loads(take_off_mission)
        altitude = 230
        transformed_mission = process_altitudes(mission, altitude)
        expected_mission = read_mission_file("test_transform_relative_altitude_with_relative_alt_in_takeoff_state2.json")
        expected_mission = json.loads(expected_mission)
        self.assertDictEqual(expected_mission, transformed_mission)

    def test_transform_relative_altitude_with_relative_alt_in_stare_position(self):
        original_mission = read_mission_file("test_transform_relative_altitude_with_relative_alt_in_stare_position.json")
        original_mission = json.loads(original_mission)
        altitude = 100.0
        transformed_mission = process_altitudes(original_mission, altitude)
        expected_mission = read_mission_file("test_transform_relative_altitude_with_relative_alt_in_stare_position2.json")
        expected_mission = json.loads(expected_mission)
        self.assertDictEqual(expected_mission, transformed_mission)
    
    def test_transform_relative_altitude_with_many_relative_alts(self):
        altitude = 230.0
        original_mission = read_mission_file("test_transform_relative_altitude_with_many_relative_alts.json")
        original_mission = json.loads(original_mission)
        transformed_mission = process_altitudes(original_mission, altitude)
        expected_mission = read_mission_file("test_transform_relative_altitude_with_many_relative_alts2.json")
        expected_mission = json.loads(expected_mission)
        self.assertDictEqual(expected_mission, transformed_mission)


import rospy