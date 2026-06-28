"""Tests for src/dr_onboard_autonomy/states/FlyWaypoints.py"""
import copy
import sys
import unittest

from unittest.mock import patch

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.states.FlyWaypoints import FlyWaypoints

from .mock_types import mock_state_kwargs


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
            "human_control",
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

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'BriarWaypoint')
    def test_how_it_builds_BriarWaypoint_with_stare_position(self, mock_BriarWaypoint):
        """test the way it builds BriarWaypoint"""
        args = mock_state_kwargs()
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

        mock_BriarWaypoint.return_value.execute.return_value = "succeeded_waypoints"
        state.execute(userdata=None)

        mock_wp = mock_BriarWaypoint.return_value

        assert mock_BriarWaypoint.call_args.args == ()
        actual_kwargs = mock_BriarWaypoint.call_args.kwargs

        print("actual_kwargs:")
        for key, value in actual_kwargs.items():
            print(f"    {key}: {value}")

        expected_kwargs_simple = dict(
            air_lease_service=args['air_lease_service'],
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
        
        expected_outcomes=['human_control', 'succeeded_waypoints', 'error', 'abort', 'rtl']

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

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'BriarWaypoint')
    def test_how_it_builds_BriarWaypoint_with_no_stare_position_and_found_outcome(self, mock_BriarWaypoint):
        """test the way it builds BriarWaypoint"""
        args = mock_state_kwargs()
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

        mock_BriarWaypoint.return_value.execute.return_value = "succeeded_waypoints"
        state.execute(userdata=None)

        mock_wp = mock_BriarWaypoint.return_value

        assert mock_BriarWaypoint.call_args.args == ()
        actual_kwargs = mock_BriarWaypoint.call_args.kwargs

        print("actual_kwargs:")
        for key, value in actual_kwargs.items():
            print(f"    {key}: {value}")

        expected_kwargs_simple = dict(
            air_lease_service=args['air_lease_service'],
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

        expected_outcomes = {
            'abort',
            'error',
            'found',
            'human_control',
            'rtl',
            'succeeded_waypoints',
        }
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

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'BriarWaypoint')
    def test_how_it_builds_BriarWaypoint_with_no_stare_position_and_found_outcome_and_target_position_outcome_and_custom_speed(self, mock_BriarWaypoint):
        """test the way it builds BriarWaypoint"""
        args = mock_state_kwargs()
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

        mock_BriarWaypoint.return_value.execute.return_value = "succeeded_waypoints"
        actual_outcome = state.execute(userdata=None)
        self.assertEqual(actual_outcome, "succeeded_waypoints")

        mock_wp = mock_BriarWaypoint.return_value

        assert mock_BriarWaypoint.call_args.args == ()
        actual_kwargs = mock_BriarWaypoint.call_args.kwargs

        print("actual_kwargs:")
        for key, value in actual_kwargs.items():
            print(f"    {key}: {value}")

        expected_kwargs_simple = dict(
            air_lease_service=args['air_lease_service'],
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
            self.assertEqual(actual_kwargs[key], value, f"BriarWaypoint was initalized with the wrong value for a kwarg. kwargs[\"{key}\"] is {actual_kwargs[key]}. But we expected  kwargs[\"{key}\"] to be {value}.")

        expected_outcomes = {
            'abort',
            'error',
            'found',
            'human_control',
            'rtl',
            'succeeded_waypoints',
            "target_position",
        }
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
        

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'BriarWaypoint')
    def test_that_unexpected_BriarWaypoint_outcomes_can_cause_FlyWaypoints_to_have_different_outcome(self, mock_BriarWaypoint):
        """test the way it builds BriarWaypoint"""
        args = mock_state_kwargs()
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

        mock_BriarWaypoint.return_value.execute.return_value = "TESTING"
        return_val = state.execute(userdata=None)

        self.assertEqual(return_val, "TESTING")

        assert mock_BriarWaypoint.call_args.args == ()
        actual_kwargs = mock_BriarWaypoint.call_args.kwargs

        print("actual_kwargs:")
        for key, value in actual_kwargs.items():
            print(f"    {key}: {value}")

        expected_kwargs_simple = dict(
            air_lease_service=args['air_lease_service'],
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
            self.assertEqual(actual_kwargs[key], value, f"BriarWaypoint was initalized with the wrong value for a kwarg. kwargs[\"{key}\"] is {actual_kwargs[key]}. But we expected  kwargs[\"{key}\"] to be {value}.")

        expected_outcomes = {
            'abort',
            'error',
            'found',
            'human_control',
            'rtl',
            'succeeded_waypoints',
            "target_position",
        }
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

    @patch.object(sys.modules['dr_onboard_autonomy.states.FlyWaypoints'], 'BriarWaypoint')
    def test_how_it_builds_BriarWaypoint_when_given_a_single_waypoint(self, mock_BriarWaypoint):
        """test the way it builds BriarWaypoint when given a single waypoint"""
        args = mock_state_kwargs()
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

        mock_BriarWaypoint.return_value.execute.return_value = "succeeded_waypoints"

        outcome = state.execute(userdata=None)
        self.assertEqual(outcome, "succeeded_waypoints")

        assert mock_BriarWaypoint.call_args.args == ()
        actual_kwargs = mock_BriarWaypoint.call_args.kwargs
        print("actual_kwargs:")
        for key, value in actual_kwargs.items():
            print(f"    {key}: {value}")

        expected_kwargs_simple = dict(
            air_lease_service=args['air_lease_service'],
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
            self.assertEqual(actual_kwargs[key], value, f"BriarWaypoint was initalized with the wrong value for a kwarg. kwargs[\"{key}\"] is {actual_kwargs[key]}. But we expected  kwargs[\"{key}\"] to be {value}.")

        expected_outcomes = {
            'abort',
            'error',
            'found',
            'human_control',
            'rtl',
            'succeeded_waypoints',
        }
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


if __name__ == '__main__':
    unittest.main()
