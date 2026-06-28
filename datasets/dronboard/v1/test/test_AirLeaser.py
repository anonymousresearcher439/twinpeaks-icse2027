import json
import sys
import unittest
from unittest.mock import (
    NonCallableMock,
    PropertyMock,
    patch
)

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.air_lease import AirLeaseService, AirTunnel
from dr_onboard_autonomy.briar_helpers import ellipsoid_to_amsl
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient


class TestAirLeaser(unittest.TestCase):
    def setUp(self):
        self.mock_air_lease_service = NonCallableMock(spec=AirLeaseService)
        self.mock_reusable_message_senders = NonCallableMock(spec=ReusableMessageSenders)
        self.mock_mqtt_client = NonCallableMock(spec=MQTTClient)
        self.mock_userdata = NonCallableMock()
        self.mock_drone = NonCallableMock(spec=MAVROSDrone)
        type(self.mock_drone).uav_name = PropertyMock(return_value="blue")


    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "TunnelRequest")
    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "RepeatTimer")
    def test_request_airlease_tunnel(
        self,
        mock_repeat_timer,
        mock_tunnel_request,
        mock_waypoint_trajectory
    ):
        from dr_onboard_autonomy.air_lease import make_waypoint_air_tunnel_func
        from dr_onboard_autonomy.states import AirLeaser
        from dr_onboard_autonomy.air_lease_requests import Request

        mock_tunnel_request_instance = Request(
            drone_id="some_drone",
            start_position=[0.0, 1.0, 2.0],
            end_position=[3.0, 4.0, 5.0],
            radius=2.0,
            request_number=0
        )
        mock_tunnel_request.return_value = mock_tunnel_request_instance

        mock_current_position = Lla(
            latitude=41.606695509416944,
            longitude=-86.35550466673673,
            altitude=230
        )
        mock_end_position = Lla(
            latitude=41.606559,
            longitude=-86.356229,
            altitude=240
        )

        def tunnel_func(lla: Lla) -> AirTunnel:
            lat, lon, alt = lla.lat, lla.lon, lla.alt
            self.assertEqual(lat, mock_current_position.lat)
            self.assertEqual(lon, mock_current_position.lon)
            self.assertEqual(alt, mock_current_position.alt)

            real_tunnel_func = make_waypoint_air_tunnel_func(end_pos=mock_end_position)
            return real_tunnel_func(lla)

        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            air_lease_service=self.mock_air_lease_service,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=self.mock_drone
        )

        air_leaser._on_position_message(message={
            "data" : mock_current_position
        })

        '''
        subsequent call to _on_position_message shouldn't send an additional air lease request
        '''
        air_leaser._on_position_message(message={
            "data" : mock_current_position
        })

        self.mock_air_lease_service.send_request.assert_called_once_with(
            request=mock_tunnel_request_instance
        )
        mock_waypoint_trajectory.return_value.start.assert_called_once_with()
        mock_waypoint_trajectory.return_value.fly_to_waypoint.assert_called_once_with(
            Lla(*ellipsoid_to_amsl(
                (
                    mock_current_position.latitude,
                    mock_current_position.longitude,
                    mock_current_position.altitude
                )
            )),
            3.0
        )


    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "MultiTunnelRequest")
    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "TunnelRequest")
    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "RepeatTimer")
    def test_request_airlease_multi_tunnel(
        self,
        mock_repeat_timer,
        mock_tunnel_request,
        mock_multi_tunnel_request,
        mock_waypoint_trajectory
    ):
        from dr_onboard_autonomy.air_lease import make_waypoint_multi_air_tunnel_func
        from dr_onboard_autonomy.states import AirLeaser
        from dr_onboard_autonomy.air_lease_requests import MultiRequest, Request

        mock_tunnel_request_instance = Request(
            drone_id="some_drone",
            start_position=[0.0, 1.0, 2.0],
            end_position=[3.0, 4.0, 5.0],
            radius=2.0,
            request_number=0
        )
        mock_tunnel_request.return_value = mock_tunnel_request_instance
        mock_multi_tunnel_request_instance = MultiRequest(
            requests=mock_tunnel_request_instance
        )
        mock_multi_tunnel_request.return_value = mock_multi_tunnel_request_instance

        mock_current_position = Lla(
            latitude=41.606695509416944,
            longitude=-86.35550466673673,
            altitude=230
        )
        mock_end_position = Lla(
            latitude=41.606559,
            longitude=-86.356229,
            altitude=240
        )

        def tunnel_func(lla: Lla) -> AirTunnel:
            lat, lon, alt = lla.lat, lla.lon, lla.alt
            self.assertEqual(lat, mock_current_position.lat)
            self.assertEqual(lon, mock_current_position.lon)
            self.assertEqual(alt, mock_current_position.alt)

            real_tunnel_func = make_waypoint_multi_air_tunnel_func(end_pos=mock_end_position)
            return real_tunnel_func(lla)

        air_leaser = AirLeaser(
            tunnel_func=tunnel_func,
            air_lease_service=self.mock_air_lease_service,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=self.mock_drone
        )

        air_leaser._on_position_message(message={
            "data" : mock_current_position
        })

        self.mock_air_lease_service.send_multi_request.assert_called_once_with(
            requests=mock_multi_tunnel_request_instance
        )


    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "TunnelRequest")
    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "RepeatTimer")
    def test_request_airlease_takeoff(
        self,
        mock_repeat_timer,
        mock_tunnel_request,
        mock_waypoint_trajectory
    ):
        from dr_onboard_autonomy.air_lease import make_waypoint_air_tunnel_func
        from dr_onboard_autonomy.states import AirLeaser
        from dr_onboard_autonomy.air_lease_requests import Request

        mock_tunnel_request_instance = Request(
            drone_id="some_drone",
            start_position=[0.0, 1.0, 2.0],
            end_position=[3.0, 4.0, 5.0],
            radius=2.0,
            request_number=0
        )
        mock_tunnel_request.return_value = mock_tunnel_request_instance

        mock_end_position = Lla(
            latitude=41.606559,
            longitude=-86.356229,
            altitude=240
        )

        mock_userdata = NonCallableMock()

        air_leaser = AirLeaser(
            tunnel_func=make_waypoint_air_tunnel_func(end_pos=mock_end_position),
            hold_position=False,
            air_lease_service=self.mock_air_lease_service,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=self.mock_drone
        )

        air_leaser._on_position_message(message={
            "data" : mock_end_position
        })

        mock_waypoint_trajectory.return_value.start.assert_not_called()
        mock_waypoint_trajectory.return_value.fly_to_waypoint.assert_not_called()


    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "RepeatTimer")
    def test_request_airlease_land(self, mock_repeat_timer, mock_waypoint_trajectory):
        from dr_onboard_autonomy.states import AirLeaser

        air_leaser = AirLeaser(
            land=True,
            hold_position=False,
            air_lease_service=self.mock_air_lease_service,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=self.mock_drone
        )

        land_outcome = air_leaser.land()

        self.assertEqual(land_outcome, "succeeded")

        self.mock_air_lease_service.land.assert_called_once_with()
        mock_waypoint_trajectory.return_value.start.assert_not_called()
        mock_waypoint_trajectory.return_value.fly_to_waypoint.assert_not_called()


    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "WaypointTrajectory")
    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "time")
    def test_request_airlease_denied(self, mock_time, mock_waypoint_trajectory):
        from dr_onboard_autonomy.states import AirLeaser

        mock_airlease_status_data = NonCallableMock()
        type(mock_airlease_status_data).payload = PropertyMock(return_value=json.dumps({
            "approved" : False
        }))

        air_leaser = AirLeaser(
            air_lease_service=self.mock_air_lease_service,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=self.mock_drone
        )

        air_leaser._on_timer(None)
        self.mock_air_lease_service.land.assert_not_called()
        self.mock_air_lease_service.send_request.assert_not_called()

        mock_time.monotonic.return_value = 0
        outcome = air_leaser._on_airlease_status_message(
            message={
                "data" : mock_airlease_status_data
            }
        )

        self.assertIsNone(outcome)
        self.assertEqual(air_leaser._time_of_response, 0)

        mock_time.monotonic.return_value = 6
        air_leaser._on_timer(None)

        self.assertIsNone(air_leaser._time_of_response)

        mock_waypoint_trajectory.return_value.stop.assert_not_called()


    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "WaypointTrajectory")
    def test_request_airlease_approved(self, mock_waypoint_trajectory):
        from dr_onboard_autonomy.states import AirLeaser

        mock_airlease_status_data = NonCallableMock()
        type(mock_airlease_status_data).payload = PropertyMock(return_value=json.dumps({
            "approved" : True
        }))

        mock_current_position = Lla(
            latitude=41.606695509416944,
            longitude=-86.35550466673673,
            altitude=230
        )

        air_leaser = AirLeaser(
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=self.mock_drone,
            hold_position=mock_current_position
        )

        air_leaser._trajectory_started = True

        outcome = air_leaser._on_airlease_status_message(
            message={
                "data" : mock_airlease_status_data
            }
        )

        self.assertEqual(outcome, "succeeded")

        mock_waypoint_trajectory.return_value.stop.assert_called_once_with()


    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "WaypointTrajectory")
    def test_request_airlease_takeoff_approved(self, mock_waypoint_trajectory):
        from dr_onboard_autonomy.states import AirLeaser

        mock_airlease_status_data = NonCallableMock()
        type(mock_airlease_status_data).payload = PropertyMock(return_value=json.dumps({
            "approved" : True
        }))

        air_leaser = AirLeaser(
            hold_position=False,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=self.mock_drone
        )

        outcome = air_leaser._on_airlease_status_message(
            message={
                "data" : mock_airlease_status_data
            }
        )

        self.assertEqual(outcome, "succeeded")

        mock_waypoint_trajectory.return_value.stop.assert_not_called()


    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "WaypointTrajectory")
    def test_communicate_route_complete(self, mock_waypoint_trajectory):
        from dr_onboard_autonomy.states import AirLeaser

        mock_current_position = Lla(
            latitude=41.606695509416944,
            longitude=-86.35550466673673,
            altitude=230
        )

        air_leaser = AirLeaser(
            air_lease_service=self.mock_air_lease_service,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=self.mock_drone
        )


        air_leaser.communicate_route_complete(current_position=mock_current_position)

        self.mock_air_lease_service.send_done.assert_called_once_with(
            position=[
                mock_current_position.lat,
                mock_current_position.lon,
                mock_current_position.alt
            ]
        )


    @patch.object(sys.modules["dr_onboard_autonomy.states.AirLeaser"], "WaypointTrajectory")
    def test_communicate_route_complete_no_position(self, mock_waypoint_trajectory):
        from dr_onboard_autonomy.states import AirLeaser

        air_leaser = AirLeaser(
            air_lease_service=self.mock_air_lease_service,
            reusable_message_senders=self.mock_reusable_message_senders,
            mqtt_client=self.mock_mqtt_client,
            drone=self.mock_drone
        )

        air_leaser.communicate_route_complete()

        self.mock_air_lease_service.send_done.assert_called_once_with()

