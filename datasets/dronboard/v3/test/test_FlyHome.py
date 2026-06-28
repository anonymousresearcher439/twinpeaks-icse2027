import unittest
import importlib

from unittest.mock import patch, NonCallableMock

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.states import FlyHome, ReadMessagesAirborne, BriarWaypoint
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory

from mock_types import mock_state_kwargs, mock_message_sender_position_message, mock_mission_builder 


flyhome_module = importlib.import_module(FlyHome.__module__)

class TestFlyHome(unittest.TestCase):
    def test_init_minimal_args(self):
        kwargs = mock_state_kwargs({})
        # note kwargs should always include a home location
        # in this test we will use the home value provided by mock_state_kwargs
        fly_home = FlyHome(**kwargs)

        expected_home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        self.assertEqual(fly_home.home.amsl.lla, expected_home.amsl.lla)

        # make sure speed got the default value since we didn't provide a speed
        self.assertEqual(fly_home.speed, 6.0)
        # make sure stare_position is None and stare_pitch is None
        self.assertIsNone(fly_home.stare_position)
        self.assertIsNone(fly_home.stare_pitch)

        # make sure fly_home.trajectory is of type WaypointTrajectory
        self.assertIsInstance(fly_home.trajectory, WaypointTrajectory)
    
    def test_init_all_args(self):
        kwargs = mock_state_kwargs({
            "speed": 5.0,
            "stare_position": {
                "latitude": 1.0,
                "longitude": 2.0,
                "altitude": 3.0,
            },
            "stare_pitch": 35.0,
        })
        fly_home = FlyHome(**kwargs)

        expected_home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        self.assertEqual(fly_home.home.amsl.lla, expected_home.amsl.lla)

        # make sure speed is 5
        self.assertEqual(fly_home.speed, 5.0)
        # make sure stare position is 1,2,3
        self.assertEqual(fly_home.stare_position.amsl.lla, Lla(1.0, 2.0, 3.0))

        # make sure stare_pitch is 35
        self.assertEqual(fly_home.stare_pitch, 35.0)

    @patch.object(flyhome_module, "ReadMessagesAirborne")
    @patch.object(flyhome_module, "BriarWaypoint")
    def test_fly_home_when_everything_goes_well(self, mock_BriarWaypoint, mock_ReadMessagesAirborne):
        builder = mock_mission_builder()
        kwargs = builder.build_kwargs({
            "speed": 5.0,
            "stare_position": {
                "latitude": 1.0,
                "longitude": 2.0,
                "altitude": 3.0,
            },
            "stare_pitch": 35.0,
            "name": "test_fly_home",
        })
        
        # setup the mock ReadMessagesAirborne state
        read_messsages_state = NonCallableMock(spec=ReadMessagesAirborne)
        read_messsages_state.execute.return_value = "succeeded"


        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        # move the drone 30 meters north, 40 meters up
        drone_pos_lla = home.ellipsoid.lla.move_ned(30, 0, -40.0)
        drone_pos_briar = BriarLla.from_lla(drone_pos_lla, is_amsl=False)

        pos_msg = mock_message_sender_position_message(drone_pos_lla.lat, drone_pos_lla.lon, drone_pos_lla.altitude)
        read_messsages_state.get.return_value = {
            "position": pos_msg['data'],
        }
        mock_ReadMessagesAirborne.return_value = read_messsages_state

        # setup the mock BriarWaypoint state
        waypoint_state = NonCallableMock(spec=BriarWaypoint)
        waypoint_state.execute.return_value = "succeeded_waypoints"
        mock_BriarWaypoint.return_value = waypoint_state

        fly_home = FlyHome(**kwargs)

        outcome = fly_home.execute(userdata={})

        self.assertEqual(outcome, "succeeded_waypoints")

        # make sure we called mock_ReadMessagesAirborne correctly
        # expected kwargs:
        #    message_names=["position"]
        #   name="test_fly_home"
        mock_ReadMessagesAirborne.assert_called_once()
        args, kwargs = mock_ReadMessagesAirborne.call_args
        self.assertIn("message_names", kwargs)
        self.assertIn("name", kwargs)
        self.assertIsInstance(kwargs["message_names"], list)
        self.assertEqual(kwargs['message_names'], ["position"])
        self.assertIn("position", kwargs["message_names"])
        # make sure we kwargs has the right name value
        self.assertIn("name", kwargs)
        self.assertEqual(kwargs["name"], "test_fly_home")

        # make sure we called mock_BriarWaypoint correctly
        # expected kwargs:
        #   waypoint=home_lat, home_long, drone_pos_lla.altitude
        #   name = "test_fly_home"
        #   speed = 5.0
        #   stare_position = 1.0, 2.0, 3.0
        #   stare_pitch = 35.0
        home_pos = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        # Our waypoint should be just above the home location at the drone's current altitude
        waypoint = BriarLla.from_args(home_pos.amsl.lla.lat, home_pos.amsl.lla.lon, drone_pos_briar.amsl.lla.altitude, is_amsl=True)
        mock_BriarWaypoint.assert_called_once()
        args, kwargs = mock_BriarWaypoint.call_args
        self.assertIn("waypoint", kwargs)
        self.assertIn("name", kwargs)
        self.assertIn("speed", kwargs)
        self.assertIn("stare_position", kwargs)
        self.assertIn("stare_pitch", kwargs)
        
        self.assertEqual(kwargs["waypoint"], waypoint.amsl.dict)
        self.assertEqual(kwargs["name"], "test_fly_home")
        self.assertEqual(kwargs["speed"], 5.0)
        self.assertEqual(kwargs["stare_position"], {
            "latitude": 1.0,
            "longitude": 2.0,
            "altitude": 3.0,
        })
        self.assertEqual(kwargs["stare_pitch"], 35.0)

    @patch.object(flyhome_module, "ReadMessagesAirborne")
    @patch.object(flyhome_module, "BriarWaypoint")
    def test_fly_home_when_read_messages_fails(self, mock_BriarWaypoint, mock_ReadMessagesAirborne):
        builder = mock_mission_builder()
        kwargs = builder.build_kwargs({
            "speed": 5.0,
            "stare_position": {
                "latitude": 1.0,
                "longitude": 2.0,
                "altitude": 3.0,
            },
            "stare_pitch": 35.0,
            "name": "test_fly_home",
        })
        
        # setup the mock ReadMessagesAirborne state
        read_messsages_state = NonCallableMock(spec=ReadMessagesAirborne)
        read_messsages_state.execute.return_value = "failsafe"

        # if we don't return from read messages then we should never call the get method
        # make sure hte get method throws an exception so the test fails
        read_messsages_state.get.side_effect = Exception("This should not be called")
        mock_ReadMessagesAirborne.return_value = read_messsages_state

        # setup the mock BriarWaypoint state
        waypoint_state = NonCallableMock(spec=BriarWaypoint)
        waypoint_state.execute.return_value = "succeeded_waypoints"
        mock_BriarWaypoint.return_value = waypoint_state

        fly_home = FlyHome(**kwargs)

        outcome = fly_home.execute(userdata={})

        self.assertEqual(outcome, "failsafe")
    
    @patch.object(flyhome_module, "ReadMessagesAirborne")
    @patch.object(flyhome_module, "BriarWaypoint")
    def test_fly_home_when_briar_waypoint_fails(self, mock_BriarWaypoint, mock_ReadMessagesAirborne):
        builder = mock_mission_builder()
        kwargs = builder.build_kwargs({
            "speed": 5.0,
            "stare_position": {
                "latitude": 1.0,
                "longitude": 2.0,
                "altitude": 3.0,
            },
            "stare_pitch": 35.0,
            "name": "test_fly_home",
        })
        
        # setup the mock ReadMessagesAirborne state
        read_messsages_state = NonCallableMock(spec=ReadMessagesAirborne)
        read_messsages_state.execute.return_value = "succeeded"

        home = BriarLla.from_dict(kwargs["home"], is_amsl=True)
        # move the drone 30 meters north, 40 meters up
        drone_pos_lla = home.ellipsoid.lla.move_ned(30, 0, -40.0)
        drone_pos_briar = BriarLla.from_lla(drone_pos_lla, is_amsl=False)

        pos_msg = mock_message_sender_position_message(drone_pos_lla.lat, drone_pos_lla.lon, drone_pos_lla.altitude)
        read_messsages_state.get.return_value = {
            "position": pos_msg['data'],
        }
        mock_ReadMessagesAirborne.return_value = read_messsages_state

        # setup the mock BriarWaypoint state
        waypoint_state = NonCallableMock(spec=BriarWaypoint)
        waypoint_state.execute.return_value = "error"
        mock_BriarWaypoint.return_value = waypoint_state

        fly_home = FlyHome(**kwargs)

        outcome = fly_home.execute(userdata={})

        self.assertEqual(outcome, "error")


