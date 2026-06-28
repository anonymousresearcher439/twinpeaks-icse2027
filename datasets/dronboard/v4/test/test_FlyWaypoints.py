"""Tests for src/dr_onboard_autonomy/states/FlyWaypoints.py"""
import copy
import sys
import unittest

from unittest.mock import patch

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.mission_helper import MissionBuilder
from dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition
from dr_onboard_autonomy.state_factory import STANDARD_TRANSITIONS_FLYING
from dr_onboard_autonomy.states.FlyWaypoints import FlyWaypoints

from .mock_types import mock_mission_builder_args, mock_state_kwargs


class TestFlyWaypoints(unittest.TestCase):
    """Test the FlyWaypoints state"""
    def test_init(self):
        """Test that you can initialize the state
        """
        args = mock_state_kwargs()
        del args['trajectory_class']

        waypoints = [
            # East side of Peppermint Field Runway
            BriarLla.from_args(41.60669513056443, -86.35561322306913, 240),

            # West side of Peppermint Field Runway
            BriarLla.from_args(41.606670532380704, -86.35656009802273, 250),

            # North side of Peppermint Field Runway (above the shed)
            BriarLla.from_args(41.6068982966982, -86.35606636226497, 260),
        ]

        args.update({
            "waypoints": [x.amsl.dict for x in waypoints],
            "stare_pitch": 99,
            "default_speed": 5,
        })

        state = FlyWaypoints(**args)
        self.assertIsNotNone(state)

        expected_outcomes = {
            "succeeded_waypoints",
            "error",
            "failsafe",
            "abort",
            "rtl"
        }

        for outcome in expected_outcomes:
            self.assertIn(outcome, state.get_registered_outcomes())

        self.assertEqual(state.stare_position, None)
        self.assertEqual(state.stare_pitch, 99)
        self.assertEqual(state.default_speed, 5)
        self.assertIsInstance(state.waypoints, list)
        for waypoint, expected_data in zip(state.waypoints, waypoints):
            self.assertAlmostEqual(waypoint.speed, 5, places=5)
            for actual, expected in zip(waypoint.position.amsl.tup, expected_data.amsl.tup):
                self.assertAlmostEqual(actual, expected, places=5)

    def test_init_each_waypoint_at_different_speed(self):
        """Test that you can specify a different speed for each waypoint
        """
        args = mock_state_kwargs()
        del args['trajectory_class']

        waypoints = [
            # East side of Peppermint Field Runway
            BriarLla.from_args(41.60669513056443, -86.35561322306913, 240),

            # West side of Peppermint Field Runway
            BriarLla.from_args(41.606670532380704, -86.35656009802273, 250),

            # North side of Peppermint Field Runway (above the shed)
            BriarLla.from_args(41.6068982966982, -86.35606636226497, 260),
        ]

        expected_speeds = [1, 2, 3]

        waypoint_args = [x.amsl.dict for x in waypoints]
        for waypoint, speed in zip(waypoint_args, expected_speeds):
            waypoint['speed'] = speed

        args.update({
            "waypoints": waypoint_args,
            "stare_pitch": 99,
            "default_speed": 5,
        })

        state = FlyWaypoints(**args)
        for waypoint, expected_waypoint, expected_speed in zip(state.waypoints, waypoints, expected_speeds):
            self.assertAlmostEqual(waypoint.speed, expected_speed, places=5)
            for actual, expected in zip(waypoint.position.amsl.tup, expected_waypoint.amsl.tup):
                self.assertAlmostEqual(actual, expected, places=5)

    def test_init_stare_position(self):
        """Test that you can specify a stare position
        """
        args = mock_state_kwargs()
        del args['trajectory_class']

        # East side of Peppermint Field Runway
        dest = BriarLla.from_args(41.60669513056443, -86.35561322306913, 240)
        stare_pos = BriarLla.from_args(41.60669513056443, -86.35561322306913, 230)
        waypoints = [
            dest.amsl.dict,
        ]
        args.update({
            "waypoints": waypoints,
            "stare_position": stare_pos.amsl.dict,
            "stare_pitch": 99,
            "default_speed": 5,
        })

        state = FlyWaypoints(**args)
        for actual, expected in zip(state.stare_position.amsl.tup, stare_pos.amsl.tup):
            self.assertAlmostEqual(actual, expected, places=5)

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'Waypoint')
    def test_how_it_builds_Waypoint_with_stare_position(self, mock_Waypoint):
        """test the way it builds Waypoint"""
        mission_builder = MissionBuilder(**mock_mission_builder_args())
        args = mission_builder.build_kwargs()
        if 'trajectory_class' in args:
            del args['trajectory_class']

        # East side of Peppermint Field Runway
        dest = BriarLla.from_args(41.60669513056443, -86.35561322306913, 240)
        stare_pos = BriarLla.from_args(41.60669513056443, -86.35561322306913, 230)
        waypoints = [
            copy.copy(dest.amsl.dict),
        ]
        args.update({
            "waypoints": waypoints,
            "stare_position": stare_pos.amsl.dict,
            "stare_pitch": 99,
            "default_speed": 5,
            "name": "FlyWaypointsTest"
        })

        state = FlyWaypoints(**args)

        mock_Waypoint.return_value.execute.return_value = "succeeded_waypoints"
        state.execute(userdata=None)

        mock_wp = mock_Waypoint.return_value

        assert mock_Waypoint.call_args.args == ()
        actual_kwargs = mock_Waypoint.call_args.kwargs

        print("actual_kwargs:")
        for key, value in actual_kwargs.items():
            print(f"    {key}: {value}")

        expected_kwargs_simple = dict(
            air_lease_protocol_fsm=args['air_lease_protocol_fsm'],
            data=args['data'],
            drone=args['drone'],
            heartbeat_handler=args['heartbeat_handler'],
            home_altitude_offset=args['home_altitude_offset'],
            local_mqtt_client=args['local_mqtt_client'],
            mqtt_client=args['mqtt_client'],
            name=args['name'],
            reusable_message_senders=args['reusable_message_senders'],
            speed=5,
            stare_pitch=99,
            uav_id=args['uav_id'],
        )
        
        # We're testing to make sure the kwargs we care about are included.
        # It's ok if there are actually more kwargs as new ones may be added
        # in the future to support new features in other states.
        for key, value in expected_kwargs_simple.items():
            self.assertEqual(actual_kwargs[key], value)
        
        expected_outcomes = set(STANDARD_TRANSITIONS_FLYING.keys())
        expected_outcomes.add('succeeded_waypoints')
        expected_outcomes.add('deadlock')


        self.assertEqual(len(actual_kwargs['outcomes']), len(expected_outcomes))
        for outcome in expected_outcomes:
            self.assertIn(outcome, actual_kwargs['outcomes'])


        actual_stare_position = BriarLla.from_dict(actual_kwargs['stare_position']).amsl.tup
        expected_stare_position=stare_pos.amsl.tup

        for actual, expected in zip(actual_stare_position, expected_stare_position):
            self.assertAlmostEqual(actual, expected, places=7)
        
        actual_waypoint = BriarLla.from_dict(actual_kwargs['waypoint']).amsl.tup
        for actual, expected in zip(actual_waypoint, dest.amsl.tup):
            self.assertAlmostEqual(actual, expected, places=7)

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'Waypoint')
    def test_how_it_builds_Waypoint_with_no_stare_position_and_found_outcome(self, mock_Waypoint):
        """test the way it builds Waypoint"""
        mission_builder = MissionBuilder(**mock_mission_builder_args())
        args = mission_builder.build_kwargs()
        if 'trajectory_class' in args:
            del args['trajectory_class']

        # East side of Peppermint Field Runway
        dest = BriarLla.from_args(41.60669513056443, -86.35561322306913, 240)
        waypoints = [
            copy.copy(dest.amsl.dict),
        ]
        args.update({
            "waypoints": waypoints,
            "stare_pitch": 99,
            "default_speed": 5,
            "name": "FlyWaypointsTest",
            "outcomes": ["found"]
        })

        state = FlyWaypoints(**args)

        mock_Waypoint.return_value.execute.return_value = "succeeded_waypoints"
        state.execute(userdata=None)

        mock_wp = mock_Waypoint.return_value

        assert mock_Waypoint.call_args.args == ()
        actual_kwargs = mock_Waypoint.call_args.kwargs

        print("actual_kwargs:")
        for key, value in actual_kwargs.items():
            print(f"    {key}: {value}")

        expected_kwargs_simple = dict(
            air_lease_protocol_fsm=args['air_lease_protocol_fsm'],
            data=args['data'],
            drone=args['drone'],
            heartbeat_handler=args['heartbeat_handler'],
            home_altitude_offset=args['home_altitude_offset'],
            local_mqtt_client=args['local_mqtt_client'],
            mqtt_client=args['mqtt_client'],
            name=args['name'],
            reusable_message_senders=args['reusable_message_senders'],
            speed=5,
            stare_pitch=99,
            uav_id=args['uav_id'],
        )
        
        # make sure the kwargs we care about are included
        for key, value in expected_kwargs_simple.items():
            self.assertEqual(actual_kwargs[key], value)

        expected_outcomes = set(STANDARD_TRANSITIONS_FLYING.keys())
        expected_outcomes.update({
            'found',
            'succeeded_waypoints',
            'deadlock'
        })
        actual_outcomes = actual_kwargs['outcomes']

        for outcome in expected_outcomes:
            self.assertIn(outcome, actual_outcomes)
        if len(actual_outcomes) != len(expected_outcomes):
            for outcome in actual_outcomes:
                self.assertIn(outcome, expected_outcomes, "actual_outcomes contains an unexpected outcome")
        self.assertEqual(len(actual_outcomes), len(expected_outcomes))

        actual_waypoint = BriarLla.from_dict(actual_kwargs['waypoint']).amsl.tup
        for actual, expected in zip(actual_waypoint, dest.amsl.tup):
            self.assertAlmostEqual(actual, expected, places=7)

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'Waypoint')
    def test_how_it_builds_Waypoint_with_no_stare_position_and_found_outcome_and_target_position_outcome_and_custom_speed(self, mock_Waypoint):
        """test the way it builds Waypoint"""
        mission_builder = MissionBuilder(**mock_mission_builder_args())
        args = mission_builder.build_kwargs()
        if 'trajectory_class' in args:
            del args['trajectory_class']

        # East side of Peppermint Field Runway
        dest = BriarLla.from_args(41.60669513056443, -86.35561322306913, 240)
        waypoints = [
            copy.copy(dest.amsl.dict),
        ]
        waypoints[0]['speed'] = 10
        args.update({
            "waypoints": waypoints,
            "stare_pitch": 99,
            "default_speed": 5,
            "name": "FlyWaypointsTest",
            "outcomes": ["found", "target_position"]
        })

        state = FlyWaypoints(**args)

        mock_Waypoint.return_value.execute.return_value = "succeeded_waypoints"
        actual_outcome = state.execute(userdata=None)
        self.assertEqual(actual_outcome, "succeeded_waypoints")

        mock_wp = mock_Waypoint.return_value

        assert mock_Waypoint.call_args.args == ()
        actual_kwargs = mock_Waypoint.call_args.kwargs

        print("actual_kwargs:")
        for key, value in actual_kwargs.items():
            print(f"    {key}: {value}")

        expected_kwargs_simple = dict(
            air_lease_protocol_fsm=args['air_lease_protocol_fsm'],
            data=args['data'],
            drone=args['drone'],
            heartbeat_handler=args['heartbeat_handler'],
            home_altitude_offset=args['home_altitude_offset'],
            local_mqtt_client=args['local_mqtt_client'],
            mqtt_client=args['mqtt_client'],
            name=args['name'],
            reusable_message_senders=args['reusable_message_senders'],
            speed=10,
            stare_pitch=99,
            uav_id=args['uav_id'],
        )

        # make sure the kwargs we care about are included
        for key, value in expected_kwargs_simple.items():
            self.assertEqual(actual_kwargs[key], value, f"Waypoint was initalized with the wrong value for a kwarg. kwargs[\"{key}\"] is {actual_kwargs[key]}. But we expected  kwargs[\"{key}\"] to be {value}.")

        expected_outcomes = set(STANDARD_TRANSITIONS_FLYING.keys())
        expected_outcomes.update({
            'found',
            'succeeded_waypoints',
            "target_position",
            'deadlock'
        })
        actual_outcomes = actual_kwargs['outcomes']

        for outcome in expected_outcomes:
            self.assertIn(outcome, actual_outcomes, "an expected outcome is missing from the actual outcomes")
        if len(actual_outcomes) != len(expected_outcomes):
            for outcome in actual_outcomes:
                self.assertIn(outcome, expected_outcomes, "the actual outcomes contains an additional unexpected outcome")
        self.assertEqual(len(actual_outcomes), len(expected_outcomes))

        actual_waypoint = BriarLla.from_dict(actual_kwargs['waypoint']).amsl.tup
        for actual, expected in zip(actual_waypoint, dest.amsl.tup):
            self.assertAlmostEqual(actual, expected, places=7)
        

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'Waypoint')
    def test_that_unexpected_Waypoint_outcomes_can_cause_FlyWaypoints_to_have_different_outcome(self, mock_Waypoint):
        """test the way it builds Waypoint"""
        mission_builder = MissionBuilder(**mock_mission_builder_args())
        args = mission_builder.build_kwargs()
        if 'trajectory_class' in args:
            del args['trajectory_class']

        # East side of Peppermint Field Runway
        dest = BriarLla.from_args(41.60669513056443, -86.35561322306913, 240)
        waypoints = [
            copy.copy(dest.amsl.dict),
        ]
        waypoints[0]['speed'] = 10
        args.update({
            "waypoints": waypoints,
            "stare_pitch": 99,
            "default_speed": 5,
            "name": "FlyWaypointsTest",
            "outcomes": ["found", "target_position"]
        })

        state = FlyWaypoints(**args)

        mock_Waypoint.return_value.execute.return_value = "TESTING"
        return_val = state.execute(userdata=None)

        self.assertEqual(return_val, "TESTING")

        assert mock_Waypoint.call_args.args == ()
        actual_kwargs = mock_Waypoint.call_args.kwargs

        print("actual_kwargs:")
        for key, value in actual_kwargs.items():
            print(f"    {key}: {value}")

        expected_kwargs_simple = dict(
            air_lease_protocol_fsm=args['air_lease_protocol_fsm'],
            data=args['data'],
            drone=args['drone'],
            heartbeat_handler=args['heartbeat_handler'],
            home=args['home'],
            home_altitude_offset=args['home_altitude_offset'],
            local_mqtt_client=args['local_mqtt_client'],
            mqtt_client=args['mqtt_client'],
            name=args['name'],
            reusable_message_senders=args['reusable_message_senders'],
            speed=10,
            stare_pitch=99,
            uav_id=args['uav_id'],
        )

        # make sure the kwargs we care about are included
        for key, value in expected_kwargs_simple.items():
            self.assertEqual(actual_kwargs[key], value, f"Waypoint was initalized with the wrong value for a kwarg. kwargs[\"{key}\"] is {actual_kwargs[key]}. But we expected  kwargs[\"{key}\"] to be {value}.")
        expected_outcomes = set(STANDARD_TRANSITIONS_FLYING.keys())
        expected_outcomes.update({
            'found',
            "target_position",
            'succeeded_waypoints',
            'deadlock',
        })

        actual_outcomes = actual_kwargs['outcomes']

        for outcome in expected_outcomes:
            self.assertIn(outcome, actual_outcomes, "an expected outcome is missing from the actual outcomes")
        if len(actual_outcomes) != len(expected_outcomes):
            for outcome in actual_outcomes:
                self.assertIn(outcome, expected_outcomes, "the actual outcomes contains an additional unexpected outcome")
        self.assertEqual(len(actual_outcomes), len(expected_outcomes))

        actual_waypoint = BriarLla.from_dict(actual_kwargs['waypoint']).amsl.tup
        for actual, expected in zip(actual_waypoint, dest.amsl.tup):
            self.assertAlmostEqual(actual, expected, places=7)

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'Waypoint')
    def test_how_it_builds_Waypoint_when_given_a_single_waypoint(self, mock_Waypoint):
        """test the way it builds Waypoint when given a single waypoint"""
        mission_builder = MissionBuilder(**mock_mission_builder_args())
        args = mission_builder.build_kwargs()
        if 'trajectory_class' in args:
            del args['trajectory_class']
        # East side of Peppermint Field Runway
        dest = BriarLla.from_args(41.60669513056443, -86.35561322306913, 240)
        args.update({
            "waypoints": copy.copy(dest.amsl.dict),
            "stare_pitch": 45,
            "default_speed": 5,
            "name": "FlyWaypointsTest",
            "outcomes": ["found"]
        })
        # in this test we're making sure it works when the `waypoints` 
        # arg is a single waypoint instead of a list
        assert isinstance(args['waypoints'], dict)
        state = FlyWaypoints(**args)

        mock_Waypoint.return_value.execute.return_value = "succeeded_waypoints"

        outcome = state.execute(userdata=None)
        self.assertEqual(outcome, "succeeded_waypoints")

        assert mock_Waypoint.call_args.args == ()
        actual_kwargs = mock_Waypoint.call_args.kwargs
        print("actual_kwargs:")
        for key, value in actual_kwargs.items():
            print(f"    {key}: {value}")

        expected_kwargs_simple = dict(
            air_lease_protocol_fsm=args['air_lease_protocol_fsm'],
            data=args['data'],
            drone=args['drone'],
            heartbeat_handler=args['heartbeat_handler'],
            home_altitude_offset=args['home_altitude_offset'],
            local_mqtt_client=args['local_mqtt_client'],
            mqtt_client=args['mqtt_client'],
            name=args['name'],
            reusable_message_senders=args['reusable_message_senders'],
            speed=5,
            stare_pitch=45,
            uav_id=args['uav_id'],
        )

        # make sure the kwargs we care about are included
        for key, value in expected_kwargs_simple.items():
            self.assertEqual(actual_kwargs[key], value, f"Waypoint was initalized with the wrong value for a kwarg. kwargs[\"{key}\"] is {actual_kwargs[key]}. But we expected  kwargs[\"{key}\"] to be {value}.")

        expected_outcomes = set(STANDARD_TRANSITIONS_FLYING.keys())
        expected_outcomes.update({
            'found',
            'succeeded_waypoints',
            'deadlock',
        })
        actual_outcomes = actual_kwargs['outcomes']

        for outcome in expected_outcomes:
            self.assertIn(outcome, actual_outcomes)
        if len(actual_outcomes) != len(expected_outcomes):
            for outcome in actual_outcomes:
                self.assertIn(outcome, expected_outcomes, "actual_outcomes contains an unexpected outcome")
        self.assertEqual(len(actual_outcomes), len(expected_outcomes))

        actual_waypoint = BriarLla.from_dict(actual_kwargs['waypoint']).amsl.tup
        for actual, expected in zip(actual_waypoint, dest.amsl.tup):
            self.assertAlmostEqual(actual, expected, places=7)

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'Waypoint')
    def test_how_we_respond_to_deadlock(self, mock_Waypoint):
        """test that we divert in case of deadlock"""
        mission_builder = MissionBuilder(**mock_mission_builder_args())
        args = mission_builder.build_kwargs()
        if 'trajectory_class' in args:
            del args['trajectory_class']
        
        pos_Lla = LlaPosition(41.606695130001185,-86.35597309878872,240.00007044513706, DATUM_REFERENCE.AMSL)
        pos_Lla = pos_Lla.to_wgs84_ellipsoid()

        args['drone'].data.location.add_position(1730828095.1277785, pos_Lla)

        # East side of Peppermint Field Runway
        dest = BriarLla.from_args(41.60669513056443, -86.35561322306913, 240)
        waypoints = [
            copy.copy(dest.amsl.dict),
        ]
        args.update({
            "waypoints": waypoints,
            "stare_pitch": 99,
            "default_speed": 5,
            "name": "FlyWaypointsTest",
            "outcomes": ["found"]
        })

        state = FlyWaypoints(**args)

        waypoint_outcomes = ["deadlock", "succeeded_waypoints", "succeeded_waypoints"]
        mock_Waypoint.return_value.execute.side_effect = waypoint_outcomes

        original_dest = BriarLla.from_args(41.60669513056443, -86.35561322306913, 240, is_amsl=True)
        pos_lla_wgs84 = pos_Lla.to_wgs84_ellipsoid()
        diversion_dest = BriarLla.from_args(pos_lla_wgs84.latitude, pos_lla_wgs84.longitude, pos_lla_wgs84.altitude,is_amsl=False).ellipsoid.lla.move_ned(0, 0, -22)
        diversion_dest = BriarLla.from_args(diversion_dest.latitude, diversion_dest.longitude, diversion_dest.altitude, is_amsl=False)
        expected_waypoints = [
            original_dest.amsl.dict,
            diversion_dest.amsl.dict,
            original_dest.amsl.dict,
        ]
        
        state.execute(userdata=None)

        for (_, wp_kwargs), expected_wp in zip(mock_Waypoint.call_args_list, expected_waypoints):
            dest_dict = wp_kwargs['waypoint']
            dest_lla = BriarLla.from_dict(dest_dict).amsl.lla
            expected_wp_lla = BriarLla.from_dict(expected_wp).amsl.lla
            self.assertLess(dest_lla.distance(expected_wp_lla), 0.0001)

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'Waypoint')
    def test_how_we_respond_to_deadlock_when_drone_is_too_high(self, mock_Waypoint):
        """test that we divert in case of deadlock"""
        mission_builder = MissionBuilder(**mock_mission_builder_args())
        args = mission_builder.build_kwargs()
        if 'trajectory_class' in args:
            del args['trajectory_class']
        home = BriarLla(args['home'], is_amsl=True)
        max_alt_wgs84 = home.ellipsoid.lla.altitude + 100
        pos_Lla = LlaPosition(41.606695130001185,-86.35597309878872,max_alt_wgs84 - 10, DATUM_REFERENCE.ELLIPSOID_WGS84)
        pos_Lla = pos_Lla.to_wgs84_ellipsoid()

        args['drone'].data.location.add_position(1730828095.1277785, pos_Lla)

        # East side of Peppermint Field Runway
        dest = BriarLla.from_args(41.60669513056443, -86.35561322306913, 240)
        waypoints = [
            copy.copy(dest.amsl.dict),
        ]
        args.update({
            "waypoints": waypoints,
            "stare_pitch": 99,
            "default_speed": 5,
            "name": "FlyWaypointsTest",
            "outcomes": ["found"]
        })

        state = FlyWaypoints(**args)

        waypoint_outcomes = ["deadlock", "succeeded_waypoints", "succeeded_waypoints"]
        mock_Waypoint.return_value.execute.side_effect = waypoint_outcomes

        original_dest = BriarLla.from_args(41.60669513056443, -86.35561322306913, 240, is_amsl=True)
        pos_lla_wgs84 = pos_Lla.to_wgs84_ellipsoid()
        diversion_dest = BriarLla.from_args(pos_lla_wgs84.latitude, pos_lla_wgs84.longitude, pos_lla_wgs84.altitude,is_amsl=False).ellipsoid.lla.move_ned(0, 0, -20)
        diversion_dest = BriarLla.from_args(diversion_dest.latitude, diversion_dest.longitude, max_alt_wgs84, is_amsl=False)
        expected_waypoints = [
            original_dest.amsl.dict,
            diversion_dest.amsl.dict,
            original_dest.amsl.dict,
        ]
        
        state.execute(userdata=None)

        for (_, wp_kwargs), expected_wp in zip(mock_Waypoint.call_args_list, expected_waypoints):
            dest_dict = wp_kwargs['waypoint']
            dest_lla = BriarLla.from_dict(dest_dict).amsl.lla
            expected_wp_lla = BriarLla.from_dict(expected_wp).amsl.lla
            self.assertLess(dest_lla.distance(expected_wp_lla), 0.0001)
if __name__ == '__main__':
    unittest.main()
