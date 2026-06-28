import unittest
import json
from unittest.mock import call, Mock, patch

import smach

from dr_onboard_autonomy.mission_helper import (
    process_altitudes,
    MissionBuilder,
)
from dr_onboard_autonomy.message_senders import ReusableMessageSenders

from .mock_types import (
    mock_base_state,
    mock_state_kwargs,
    mock_mission_builder_args,
    mock_mqtt_client,
    mock_state_factory,
    read_mission_file,
)


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

        builder = MissionBuilder(
            state_factory=mock_state_factory(),
            uav_id="test_drone",
            mqtt_client=mock_args["mqtt_client"],
            local_mqtt_client=mock_args["local_mqtt_client"],
            reusable_message_senders=mock_args["reusable_message_senders"],
            drone=mock_args["drone"],
            air_lease_service=mock_args["air_lease_service"],
        )

        self.assertIsInstance(builder, MissionBuilder)
        builder._setup_tasks.clear()
        output_args = builder.build_kwargs({})
        expected_args = {
            "air_lease_service": mock_args["air_lease_service"], 
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
        }
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

    
    @patch("dr_onboard_autonomy.mission_helper.MQTTClient")
    def test_mqtt_init(self, MQTTClient):
        """
        Test the initialization of a MissionBuilder with MQTT arguments specified as host strings.
        The MissionBuilder is expected to initialize the MQTTClient objects and connect.
        """
        def make_mock_mqtt_client(uav_name, broker_address):
            mqtt = mock_mqtt_client()
            mqtt.uav_name = uav_name
            mqtt.broker_address = broker_address
            return mqtt
        
        MQTTClient.side_effect = make_mock_mqtt_client

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
            air_lease_service=mock_args["air_lease_service"],
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
        mission = builder.build_task(mission_json)
        mission.check_consistency()
        self.assertIsInstance(mission, smach.StateMachine)


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


