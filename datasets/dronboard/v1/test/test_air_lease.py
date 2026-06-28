import unittest
from unittest.mock import (
    NonCallableMock,
    PropertyMock
)
from droneresponse_mathtools import Lla

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.mqtt_client import MQTTClient


class TestAirTunnels(unittest.TestCase):
    def test_make_circle_air_tunnel_func(self):
        from src.dr_onboard_autonomy.air_lease import make_circle_air_tunnel_func
        
        mock_center_pos = Lla(39.813126, -104.865990, 5600)
        mock_current_pos = Lla(39.813535, -104.864489, 5600)
        expected_buffer = 5.0

        air_tunnel_func = make_circle_air_tunnel_func(mock_center_pos)

        tunnel_start_pos, tunnel_end_pos, tunnel_radius = air_tunnel_func(mock_current_pos)

        self.assertEqual(tunnel_start_pos, mock_center_pos.move_ned(0, 0, -expected_buffer))
        self.assertEqual(tunnel_end_pos, mock_center_pos.move_ned(0, 0, expected_buffer))
        self.assertAlmostEqual(tunnel_radius, 141.43, places=2)


    def test_make_waypoint_air_tunnel_func(self):
        from src.dr_onboard_autonomy.air_lease import make_waypoint_air_tunnel_func

        expected_air_tunnel_radius = 5.0

        mock_current_pos = Lla(39.813126, -104.865990, 5250)
        mock_end_pos = Lla(39.813535, -104.864489, 5600)

        find_waypoint_air_tunnel = make_waypoint_air_tunnel_func(mock_end_pos)

        tunnel_start_pos, tunnel_end_pos, tunnel_radius = find_waypoint_air_tunnel(mock_current_pos)

        self.assertEqual(tunnel_start_pos, mock_current_pos)
        self.assertEqual(tunnel_end_pos, mock_end_pos)
        self.assertEqual(tunnel_radius, expected_air_tunnel_radius)


    def test_make_waypoint_multi_air_tunnel_func(self):
        from src.dr_onboard_autonomy.air_lease import make_waypoint_multi_air_tunnel_func

        expected_air_tunnel_radius = 5.0

        mock_current_pos = Lla(39.813126, -104.865990, 5250)
        mock_end_pos = mock_current_pos.move_ned(0.0, 21.4, 0.0)

        find_waypoint_air_tunnel = make_waypoint_multi_air_tunnel_func(mock_end_pos)

        air_tunnels = find_waypoint_air_tunnel(mock_current_pos)

        self.assertEqual(len(air_tunnels), 3)
        for i in range(3):
            self.assertAlmostEqual(air_tunnels[0].start_pos[i], mock_current_pos[i])
            self.assertAlmostEqual(air_tunnels[0].end_pos[i], mock_current_pos.move_ned(0.0, 8.0, 0.0)[i])
            self.assertAlmostEqual(air_tunnels[1].start_pos[i], mock_current_pos.move_ned(0.0, 8.0, 0.0)[i])
            self.assertAlmostEqual(air_tunnels[1].end_pos[i], mock_current_pos.move_ned(0.0, 16.0, 0.0)[i])
            self.assertAlmostEqual(air_tunnels[2].start_pos[i], mock_current_pos.move_ned(0.0, 16.0, 0.0)[i])
            self.assertAlmostEqual(air_tunnels[2].end_pos[i], mock_current_pos.move_ned(0.0, 21.4, 0.0)[i])

        self.assertAlmostEqual(air_tunnels[1].radius, expected_air_tunnel_radius)
        self.assertAlmostEqual(air_tunnels[0].radius, expected_air_tunnel_radius)
        self.assertAlmostEqual(air_tunnels[2].radius, expected_air_tunnel_radius)


    def test_make_waypoint_multi_air_tunnel_func_at_least_one_tunnel(self):
        from src.dr_onboard_autonomy.air_lease import make_waypoint_multi_air_tunnel_func

        mock_current_pos = Lla(39.813126, -104.865990, 5250)
        mock_end_pos = mock_current_pos.move_ned(0.0, 0.5, 0.0)

        find_waypoint_air_tunnel = make_waypoint_multi_air_tunnel_func(mock_end_pos)

        air_tunnels = find_waypoint_air_tunnel(mock_current_pos)

        self.assertEqual(len(air_tunnels), 1)


    def test_make_buffer_air_tunnel_func(self):
        from src.dr_onboard_autonomy.air_lease import make_buffer_air_tunnel_func

        mock_current_pos = Lla(39.813535, -104.864489, 5600)
        expected_buffer = 5.0

        air_tunnel_func = make_buffer_air_tunnel_func()

        tunnel_start_pos, tunnel_end_pos, tunnel_radius = air_tunnel_func(mock_current_pos)

        self.assertEqual(tunnel_start_pos, mock_current_pos.move_ned(0, 0, -expected_buffer / 2))
        self.assertEqual(tunnel_end_pos, mock_current_pos.move_ned(0, 0, expected_buffer / 2))
        self.assertEqual(tunnel_radius, 5.0)


class TestAirLeaseService(unittest.TestCase):
    def setUp(self):
        self.mock_mqtt_client = NonCallableMock(spec=MQTTClient)
        self.mock_drone = NonCallableMock(spec=MAVROSDrone)
        type(self.mock_drone).uav_name = PropertyMock(return_value="blue")


    def test_send_request(self):
        from dr_onboard_autonomy.air_lease import AirLeaseService
        from dr_onboard_autonomy.air_lease_requests import Request as TunnelRequest

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
        mock_radius = 5

        air_lease_service = AirLeaseService(
            uav_id=self.mock_drone.uav_name,
            mqtt_client=self.mock_mqtt_client
        )

        tunnel_request = TunnelRequest(
            drone_id="some random drone id",
            start_position=[
                mock_current_position.lat,
                mock_current_position.lon,
                mock_current_position.alt
            ],
            end_position=[
                mock_end_position.lat,
                mock_end_position.lon,
                mock_end_position.alt
            ],
            radius=mock_radius,
            request_number=0
        )

        air_lease_service.send_request(request=tunnel_request)

        publish_arguments = self.mock_mqtt_client.publish.call_args.kwargs
        self.assertEqual(publish_arguments["topic"], "airlease/request")
        self.assertEqual(
            publish_arguments["data"]["drone_id"],
            self.mock_drone.uav_name
        )
        self.assertEqual(
            publish_arguments["data"]["start_position"],
            [mock_current_position.lat, mock_current_position.lon, mock_current_position.alt]
        )
        self.assertEqual(
            publish_arguments["data"]["request_number"],
            1
        )
        self.assertEqual(
            publish_arguments["data"]["end_position"],
            [mock_end_position.lat, mock_end_position.lon, mock_end_position.alt]
        )


    def test_send_multi_request(self):
        from dr_onboard_autonomy.air_lease import AirLeaseService
        from dr_onboard_autonomy.air_lease_requests import MultiRequest as MultiTunnelRequest
        from dr_onboard_autonomy.air_lease_requests import Request as TunnelRequest

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
        mock_radius = 5

        air_lease_service = AirLeaseService(
            uav_id=self.mock_drone.uav_name,
            mqtt_client=self.mock_mqtt_client
        )

        tunnel_requests = [
            TunnelRequest(
                drone_id="some random drone id",
                start_position=[
                    mock_current_position.lat,
                    mock_current_position.lon,
                    mock_current_position.alt
                ],
                end_position=[
                    mock_end_position.lat,
                    mock_end_position.lon,
                    mock_end_position.alt
                ],
                radius=mock_radius,
                request_number=0
            ),
            TunnelRequest(
                drone_id="some random drone id",
                start_position=[
                    mock_current_position.lat,
                    mock_current_position.lon,
                    mock_current_position.alt
                ],
                end_position=[
                    mock_end_position.lat,
                    mock_end_position.lon,
                    mock_end_position.alt
                ],
                radius=mock_radius,
                request_number=0
            )
        ]

        air_lease_service.send_multi_request(requests=MultiTunnelRequest(requests=tunnel_requests))

        publish_arguments = self.mock_mqtt_client.publish.call_args.kwargs
        self.assertEqual(publish_arguments["topic"], "airlease/request")
        for i, request in enumerate(tunnel_requests):
            self.assertEqual(
                publish_arguments["data"]["requests"][i]["drone_id"],
                self.mock_drone.uav_name
            )
            self.assertEqual(
                publish_arguments["data"]["requests"][i]["start_position"],
                request.start_position
            )
            self.assertEqual(
                publish_arguments["data"]["requests"][i]["end_position"],
                request.end_position
            )
            self.assertEqual(
                publish_arguments["data"]["requests"][i]["request_number"],
                i + 1
            )

            expected_last_request = TunnelRequest(
                drone_id=self.mock_drone.uav_name,
                start_position=tunnel_requests[0].start_position,
                end_position=tunnel_requests[0].end_position,
                radius=tunnel_requests[0].radius,
                request_number=2
            )
            self.assertEqual(air_lease_service._last_request, expected_last_request)


    def test_send_done(self):
        from dr_onboard_autonomy.air_lease import AirLeaseService
        from dr_onboard_autonomy.air_lease_requests import Request as TunnelRequest

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
        mock_radius = 5

        air_lease_service = AirLeaseService(
            uav_id=self.mock_drone.uav_name,
            mqtt_client=self.mock_mqtt_client
        )

        tunnel_request = TunnelRequest(
            drone_id="some random drone id",
            start_position=[
                mock_current_position.lat,
                mock_current_position.lon,
                mock_current_position.alt
            ],
            end_position=[
                mock_end_position.lat,
                mock_end_position.lon,
                mock_end_position.alt
            ],
            radius=mock_radius,
            request_number=0
        )

        air_lease_service.send_request(request=tunnel_request)

        air_lease_service.send_done(position=[
            mock_current_position.lat,
            mock_current_position.lon,
            mock_current_position.alt
        ])

        publish_arguments = self.mock_mqtt_client.publish.call_args.kwargs
        self.assertEqual(publish_arguments["topic"], "airlease/hover")
        self.assertEqual(
            publish_arguments["data"]["drone_id"],
            self.mock_drone.uav_name
        )
        self.assertEqual(
            publish_arguments["data"]["current_position"],
            [mock_current_position.lat, mock_current_position.lon, mock_current_position.alt]
        )
        self.assertEqual(
            publish_arguments["data"]["request_number"],
            1
        )


    def test_send_done_no_position_provided(self):
        from dr_onboard_autonomy.air_lease import AirLeaseService
        from dr_onboard_autonomy.air_lease_requests import Request as TunnelRequest

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
        mock_radius = 5

        air_lease_service = AirLeaseService(
            uav_id=self.mock_drone.uav_name,
            mqtt_client=self.mock_mqtt_client
        )

        tunnel_request = TunnelRequest(
            drone_id="some random drone id",
            start_position=[
                mock_current_position.lat,
                mock_current_position.lon,
                mock_current_position.alt
            ],
            end_position=[
                mock_end_position.lat,
                mock_end_position.lon,
                mock_end_position.alt
            ],
            radius=mock_radius,
            request_number=0
        )

        air_lease_service.send_request(request=tunnel_request)

        air_lease_service.send_done()

        publish_arguments = self.mock_mqtt_client.publish.call_args.kwargs
        self.assertEqual(
            publish_arguments["data"]["current_position"],
            [mock_end_position.lat, mock_end_position.lon, mock_end_position.alt]
        )


    def test_send_done_no_previous_request_send(self):
        from dr_onboard_autonomy.air_lease import AirLeaseService

        air_lease_service = AirLeaseService(
            uav_id=self.mock_drone.uav_name,
            mqtt_client=self.mock_mqtt_client
        )

        air_lease_service.send_done()

        self.mock_mqtt_client.publish.assert_not_called()


    def test_land(self):
        from dr_onboard_autonomy.air_lease import AirLeaseService

        air_lease_service = AirLeaseService(
            uav_id=self.mock_drone.uav_name,
            mqtt_client=self.mock_mqtt_client
        )

        air_lease_service.land()

        publish_arguments = self.mock_mqtt_client.publish.call_args.kwargs
        self.assertEqual(publish_arguments["topic"], "airlease/land")
        self.assertEqual(
            publish_arguments["data"]["drone_id"],
            self.mock_drone.uav_name
        )
        self.assertEqual(
            publish_arguments["data"]["request_number"],
            0
        )


    def test_send_cleanup(self):
        from dr_onboard_autonomy.air_lease import AirLeaseService

        air_lease_service = AirLeaseService(
            uav_id=self.mock_drone.uav_name,
            mqtt_client=self.mock_mqtt_client
        )

        mock_current_position = Lla(
            latitude=41.606695509416944,
            longitude=-86.35550466673673,
            altitude=230
        )

        air_lease_service.send_cleanup(
            position=(
                mock_current_position.lat,
                mock_current_position.lon,
                mock_current_position.alt
            )
        )

        publish_arguments = self.mock_mqtt_client.publish.call_args.kwargs
        self.assertEqual(publish_arguments["topic"], "airlease/cleanup")
        self.assertEqual(
            publish_arguments["data"]["drone_id"],
            self.mock_drone.uav_name
        )
        self.assertEqual(
            publish_arguments["data"]["position"],
            (mock_current_position.lat, mock_current_position.lon, mock_current_position.alt)
        )

