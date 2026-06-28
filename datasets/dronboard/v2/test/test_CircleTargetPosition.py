from unittest import TestCase
from unittest.mock import NonCallableMock, patch
import sys

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.briar_helpers import find_circle_start
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict
from dr_onboard_autonomy.models import ALTITUDE_REFERENCE, LlaPosition
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.mqtt_client import MQTTClient
from test.mock_types import mock_copter_drone


class TestCircleTargetPosition(TestCase):
    def setUp(self):
        self.drone = mock_copter_drone()
        self.drone_b = mock_copter_drone()

        # add Mock position data
        self.drone_pos = LlaPosition(
            latitude=41.51931934140587,
            longitude=-86.23969254550647,
            altitude=245.0,
            altitude_ref=ALTITUDE_REFERENCE.AMSL
        )

        self.drone.data.location.add_position(1695232820.587933, self.drone_pos)

        self.mock_msg_sender = NonCallableMock(spec=AbstractMessageSender)
        self.reusable_msg_senders = NonCallableMock(spec=ReusableMessageSenders)

        self.mock_mqtt = NonCallableMock(spec=MQTTClient)
        self.mock_mqtt_local = NonCallableMock(spec=MQTTClient)

        self.mock_userdata = NonCallableMock()


    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleTargetPosition"], "BriarCircle")
    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleTargetPosition"], "BriarWaypoint")
    def test_CircleTargetPosition_target_circled(
        self,
        mock_BriarWaypoint,
        mock_BriarCircle
    ):
        from dr_onboard_autonomy.states import CircleTargetPosition

        mock_target_position = LlaPosition(
            latitude=39.747394,
            longitude=-105.044845,
            altitude=1578.56,
            altitude_ref=ALTITUDE_REFERENCE.AMSL
        )
        mock_target_position.to_wgs84_ellipsoid()

        self.drone.data.target_position = mock_target_position
        eastward_drone_pos = Lla(
            mock_target_position.latitude,
            mock_target_position.longitude,
            mock_target_position.altitude).move_ned(0, 100, -100)
        
        # TODO: there is an exisiting position already in place from test_BriarHover
        # NOTE: with the exisiting position in place, an attempt to add a new position with 
        # timestamp older than existing will fail without warning and return None
        self.drone.data.location.add_position(1695232830.0, LlaPosition(
            eastward_drone_pos.latitude,
            eastward_drone_pos.longitude,
            eastward_drone_pos.altitude,
            ALTITUDE_REFERENCE.ELLIPSOID_WGS84
        ))

        mock_BriarWaypoint.return_value.execute.return_value = "succeeded_waypoints"
        mock_BriarCircle.return_value.execute.return_value = "succeeded_circle"

        mock_target_circle_radius = 35.0
        mock_target_circle_height = 22.0
        mock_approach_speed = 3.5
        mock_circle_speed = 5.0

        mock_kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "outcomes": ["error"]
        }

        circle_target_position = CircleTargetPosition(
            target_circle_radius=mock_target_circle_radius,
            target_circle_height=mock_target_circle_height,
            target_approach_speed=mock_approach_speed,
            circle_speed=mock_circle_speed,
            **mock_kwargs
        )

        outcome = circle_target_position.execute(userdata=self.mock_userdata)

        self.assertEqual(outcome, "succeeded_circle")

        # altitude in AMSL
        expected_circle_start_position = LlaDict(
            latitude=39.74739399928208,
            longitude=-105.04443673753691,
            altitude=1600.5600958759092
        )
        
        kwargs_BriarWaypoint_alt = mock_BriarWaypoint.call_args_list[0].kwargs
        expect_alt_waypoint_position = BriarLla.from_lla(eastward_drone_pos, is_amsl=False).amsl.dict
        expect_alt_waypoint_position["altitude"]=1600.56701

        self.assertAlmostEqual(
            kwargs_BriarWaypoint_alt["waypoint"]["latitude"],
            expect_alt_waypoint_position["latitude"]
        )
        self.assertAlmostEqual(
            kwargs_BriarWaypoint_alt["waypoint"]["longitude"],
            expect_alt_waypoint_position["longitude"]
        )
        self.assertAlmostEqual(
            kwargs_BriarWaypoint_alt["waypoint"]["altitude"],
            expect_alt_waypoint_position["altitude"],
            2
        )

        kwargs_BriarWaypoint = mock_BriarWaypoint.call_args_list[1].kwargs

        mock_target_position.to_amsl()
        self.assertAlmostEqual(
            kwargs_BriarWaypoint["waypoint"]["latitude"],
            expected_circle_start_position["latitude"]
        )
        self.assertAlmostEqual(
            kwargs_BriarWaypoint["waypoint"]["longitude"],
            expected_circle_start_position["longitude"]
        )
        self.assertAlmostEqual(
            kwargs_BriarWaypoint["waypoint"]["altitude"],
            expected_circle_start_position["altitude"],
            2
        )
        self.assertEqual(
            kwargs_BriarWaypoint["stare_position"],
            LlaDict(
                latitude=mock_target_position.latitude,
                longitude=mock_target_position.longitude,
                altitude=mock_target_position.altitude
            )
        )
        self.assertEqual(
            kwargs_BriarWaypoint["speed"],
            mock_approach_speed
        )
        '''check that kwargs passed through to BriarWaypoint
        '''
        self.assertEqual(
            kwargs_BriarWaypoint["uav_id"],
            mock_kwargs["uav_id"]
        )

        kwargs_BriarCircle = mock_BriarCircle.call_args.kwargs

        self.assertAlmostEqual(
            kwargs_BriarCircle["center_position"],
            LlaDict(
                latitude=mock_target_position.latitude,
                longitude=mock_target_position.longitude,
                altitude=mock_target_position.altitude
            )
        )
        self.assertAlmostEqual(
            kwargs_BriarCircle["stare_position"],
            LlaDict(
                latitude=mock_target_position.latitude,
                longitude=mock_target_position.longitude,
                altitude=mock_target_position.altitude
            )
        )
        self.assertAlmostEqual(
            kwargs_BriarCircle["speed"],
            mock_circle_speed
        )
        '''check that kwargs passed through to BriarCircle
        '''
        self.assertEqual(
            kwargs_BriarCircle["uav_id"],
            mock_kwargs["uav_id"]
        )


    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleTargetPosition"], "BriarWaypoint")
    def test_CircleTargetPosition_BriarWaypoint_fails(
        self,
        mock_BriarWaypoint
    ):
        from dr_onboard_autonomy.states import CircleTargetPosition

        mock_target_position = LlaPosition(
            latitude=39.747394,
            longitude=-105.044845,
            altitude=1578.56,
            altitude_ref=ALTITUDE_REFERENCE.AMSL
        )
        mock_target_position.to_wgs84_ellipsoid()
        self.drone.data.target_position = mock_target_position

        mock_BriarWaypoint.return_value.execute.return_value = "some_failure"

        mock_target_circle_radius = 35.0
        mock_target_circle_height = 22.0
        mock_approach_speed = 3.5
        mock_circle_speed = 5.0

        kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "outcomes": ["error"]
        }

        circle_target_position = CircleTargetPosition(
            target_circle_radius=mock_target_circle_radius,
            target_circle_height=mock_target_circle_height,
            target_approach_speed=mock_approach_speed,
            circle_speed=mock_circle_speed,
            **kwargs
        )

        self.assertEqual(
            circle_target_position.execute(userdata=self.mock_userdata),
            "some_failure"
        )


    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleTargetPosition"], "BriarCircle")
    @patch.object(sys.modules["dr_onboard_autonomy.states.CircleTargetPosition"], "BriarWaypoint")
    def test_CircleTargetPosition_BriarCircle_fails(
        self,
        mock_BriarWaypoint,
        mock_BriarCircle
    ):
        from dr_onboard_autonomy.states import CircleTargetPosition

        mock_target_position = LlaPosition(
            latitude=39.747394,
            longitude=-105.044845,
            altitude=1578.56,
            altitude_ref=ALTITUDE_REFERENCE.AMSL
        )
        mock_target_position.to_wgs84_ellipsoid()
        self.drone.data.target_position = mock_target_position

        mock_BriarWaypoint.return_value.execute.return_value = "succeeded_waypoints"
        mock_BriarCircle.return_value.execute.return_value = "some_failure"

        mock_target_circle_radius = 35.0
        mock_target_circle_height = 22.0
        mock_approach_speed = 3.5
        mock_circle_speed = 5.0

        mock_kwargs = {
            "drone": self.drone,
            "reusable_message_senders": self.reusable_msg_senders,
            "uav_id": "uav_id",
            "mqtt_client": self.mock_mqtt,
            "local_mqtt_client": self.mock_mqtt_local,
            "outcomes": ["error"]
        }

        circle_target_position = CircleTargetPosition(
            target_circle_radius=mock_target_circle_radius,
            target_circle_height=mock_target_circle_height,
            target_approach_speed=mock_approach_speed,
            circle_speed=mock_circle_speed,
            **mock_kwargs
        )

        self.assertEqual(
            circle_target_position.execute(userdata=self.mock_userdata),
            "some_failure"
        )

    def test_find_circle_start_pos(self):
        """basic test of find_circle_start"""
        drone_pos = self.drone.data.location.get_position()
        target_pos: BriarLla = BriarLla.from_args(
            latitude=41.51909245706908,
            longitude=-86.23943746139044,
            altitude=230.0,
            is_amsl=True
        )

        drone_pos.to_wgs84_ellipsoid()
        start_pos: BriarLla = find_circle_start(
            current_pos=Lla(
                drone_pos.latitude,
                drone_pos.longitude,
                drone_pos.altitude
            ),
            target_pos=target_pos.ellipsoid.lla,
            target_radius=10.0,
            target_circle_height=15.0,
        )
        alt = start_pos.amsl.lla.altitude
        # the target is at 230, the height is 15
        # the start pos should be at 230 + 15
        self.assertAlmostEqual(alt, 245.0, places=2) # 2 places -> accuracy to a centimeter

    def test_rare_edge_case_where_drone_is_directly_over_the_target(self):
        """There is a rare edge case in _get_circle_start_lat_lon2
        If the drone is directly above the target location, we'll get a divide by zero error if we don't explicitly handle this case
        In this case, the drone can start the circle anywhere. But this test assumes it was start east of the target.
        """
        RADIUS = 10.0 # meters
        HEIGHT = 10.0 # meters

        drone_lla = Lla(0, 0, 10) # the drone is 10 meters up
        target_lla = Lla(0, 0, 0) # target is directly below

        drone_pos: BriarLla = BriarLla.from_lla(drone_lla, is_amsl=False)
        target_pos: BriarLla = BriarLla.from_lla(target_lla, is_amsl=False)


        actual_start_pos: BriarLla = find_circle_start(
            current_pos=drone_pos.ellipsoid.lla,
            target_pos=target_pos.ellipsoid.lla,
            target_radius=RADIUS,
            target_circle_height=HEIGHT,
        )

        expected_start_lla = target_lla.move_ned(0, RADIUS, -HEIGHT)
        expected_start_pos: BriarLla = BriarLla.from_lla(
            expected_start_lla,
            is_amsl=False
        )

        self.assertDictEqual(
            actual_start_pos.ellipsoid.dict,
            expected_start_pos.ellipsoid.dict
        )

        alt = actual_start_pos.ellipsoid.lla.altitude
        self.assertAlmostEqual(alt, 10.0, places=2) # 2 places -> accuracy to a centimeter
