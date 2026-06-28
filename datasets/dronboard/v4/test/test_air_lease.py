import unittest
from unittest.mock import (
    NonCallableMock,
    PropertyMock,
    patch,
)
from droneresponse_mathtools import Lla

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.models.kinematics import LlaPosition
from dr_onboard_autonomy.mqtt_client import MQTTClient


class TestAirTunnels(unittest.TestCase):
    def test_make_circle_air_tunnel_func(self):
        from dr_onboard_autonomy.airlease import make_circle_air_tunnel_func
        
        mock_center_pos = Lla(39.813126, -104.865990, 5600)
        mock_current_pos = Lla(39.813535, -104.864489, 5600)
        expected_buffer = 5.0

        air_tunnel_func = make_circle_air_tunnel_func(LlaPosition.from_lla(mock_center_pos, is_amsl=False))

        airspace_volume = air_tunnel_func(LlaPosition.from_lla(mock_current_pos, is_amsl=False))
        self.assertEqual(len(airspace_volume), 1)
        segment = airspace_volume[0]
        tunnel_start_pos, tunnel_end_pos, tunnel_radius = segment.start, segment.end, segment.radius

        actual_start = Lla(tunnel_start_pos.latitude, tunnel_start_pos.longitude, tunnel_start_pos.altitude)
        actual_end = Lla(tunnel_end_pos.latitude, tunnel_end_pos.longitude, tunnel_end_pos.altitude)

        expected_start = mock_center_pos.move_ned(0, 0, -expected_buffer)
        expected_end = mock_center_pos.move_ned(0, 0, expected_buffer)
        self.assertLess(expected_start.distance(actual_start), 0.01)
        self.assertLess(expected_end.distance(actual_end), 0.01)

        self.assertAlmostEqual(tunnel_radius, 141.43, places=2)


    def test_make_waypoint_air_tunnel_func(self):
        from dr_onboard_autonomy.airlease import make_waypoint_air_tunnel_func

        expected_air_tunnel_radius = 5.0

        mock_current_pos = Lla(39.813126, -104.865990, 5250)
        mock_end_pos = Lla(39.813535, -104.864489, 5600)

        find_waypoint_air_tunnel = make_waypoint_air_tunnel_func(LlaPosition.from_lla(mock_end_pos, is_amsl=False))
        volume = find_waypoint_air_tunnel(LlaPosition.from_lla(mock_current_pos, is_amsl=False))
        self.assertEqual(len(volume), 1)
        segment = volume[0]
        tunnel_start_pos, tunnel_end_pos, tunnel_radius = segment.start, segment.end, segment.radius
        actual_start = Lla(tunnel_start_pos.latitude, tunnel_start_pos.longitude, tunnel_start_pos.altitude)
        actual_end = Lla(tunnel_end_pos.latitude, tunnel_end_pos.longitude, tunnel_end_pos.altitude)

        self.assertEqual(tunnel_radius, expected_air_tunnel_radius)
        self.assertLess(actual_start.distance(mock_current_pos), 0.01)
        self.assertLess(actual_end.distance(mock_end_pos), 0.01)

    def test_make_waypoint_multi_air_tunnel_func(self):
        from dr_onboard_autonomy.airlease import make_waypoint_multi_air_tunnel_func

        expected_air_tunnel_radius = 5.0

        mock_current_pos = Lla(39.813126, -104.865990, 5250)
        drone_pos = LlaPosition.from_lla(mock_current_pos, is_amsl=False)
        mock_end_pos = mock_current_pos.move_ned(0.0, 21.4, 0.0)
        waypoint = LlaPosition.from_lla(mock_end_pos, is_amsl=False)

        find_waypoint_air_tunnel = make_waypoint_multi_air_tunnel_func(waypoint)

        volume = find_waypoint_air_tunnel(drone_pos)
        start_llas = []
        end_llas = []
        for segment in volume:
            start = segment.start.to_wgs84_ellipsoid()
            start = Lla(start.latitude, start.longitude, start.altitude)
            start_llas.append(start)
            end = segment.end.to_wgs84_ellipsoid()
            end = Lla(end.latitude, end.longitude, end.altitude)
            end_llas.append(end)

        self.assertEqual(len(volume), 3)
        for i in range(3):
            self.assertAlmostEqual(start_llas[0][i], mock_current_pos[i])
            self.assertAlmostEqual(end_llas[0][i], mock_current_pos.move_ned(0.0, 8.0, 0.0)[i])
            self.assertAlmostEqual(start_llas[1][i], mock_current_pos.move_ned(0.0, 8.0, 0.0)[i])
            self.assertAlmostEqual(end_llas[1][i], mock_current_pos.move_ned(0.0, 16.0, 0.0)[i])
            self.assertAlmostEqual(start_llas[2][i], mock_current_pos.move_ned(0.0, 16.0, 0.0)[i])
            self.assertAlmostEqual(end_llas[2][i], mock_current_pos.move_ned(0.0, 21.4, 0.0)[i])

        self.assertAlmostEqual(volume[1].radius, expected_air_tunnel_radius)
        self.assertAlmostEqual(volume[0].radius, expected_air_tunnel_radius)
        self.assertAlmostEqual(volume[2].radius, expected_air_tunnel_radius)


    def test_make_waypoint_multi_air_tunnel_func_at_least_one_tunnel(self):
        from dr_onboard_autonomy.airlease import make_waypoint_multi_air_tunnel_func

        mock_current_pos = Lla(39.813126, -104.865990, 5250)
        drone_pos = LlaPosition.from_lla(mock_current_pos, is_amsl=False)
        mock_end_pos = mock_current_pos.move_ned(0.0, 0.5, 0.0)
        waypoint = LlaPosition.from_lla(mock_end_pos, is_amsl=False)

        find_waypoint_air_tunnel = make_waypoint_multi_air_tunnel_func(waypoint)

        air_tunnels = find_waypoint_air_tunnel(drone_pos)

        self.assertEqual(len(air_tunnels), 1)


    def test_make_buffer_air_tunnel_func(self):
        from dr_onboard_autonomy.airlease import make_buffer_air_tunnel_func

        mock_current_pos = Lla(39.813535, -104.864489, 5600)
        expected_buffer = 5.0

        air_tunnel_func = make_buffer_air_tunnel_func()
        drone_pos = LlaPosition.from_lla(mock_current_pos, is_amsl=False)
        actual_airspace_volume = air_tunnel_func(drone_pos)
        self.assertEqual(len(actual_airspace_volume), 1)
        segment = actual_airspace_volume[0]
        tunnel_start_pos, tunnel_end_pos, tunnel_radius = segment.start.to_wgs84_ellipsoid(), segment.end.to_wgs84_ellipsoid(), segment.radius

        actual_start = Lla(tunnel_start_pos.latitude, tunnel_start_pos.longitude, tunnel_start_pos.altitude)
        actual_end = Lla(tunnel_end_pos.latitude, tunnel_end_pos.longitude, tunnel_end_pos.altitude)

        expected_start = mock_current_pos.move_ned(0, 0, expected_buffer / 2)
        expected_end = mock_current_pos.move_ned(0, 0, -expected_buffer / 2)
        self.assertEqual(tunnel_radius, 5.0)
        self.assertLess(expected_start.distance(actual_start), 0.01)
        self.assertLess(expected_end.distance(actual_end), 0.01)

class TestAirLeaseService(unittest.TestCase):
    def setUp(self):
        self.mock_mqtt_client = NonCallableMock(spec=MQTTClient)
        self.mock_drone = NonCallableMock(spec=MAVROSDrone)
        type(self.mock_drone).uav_name = PropertyMock(return_value="blue")

    def test_send_done_no_previous_request_send(self):
        from dr_onboard_autonomy.air_lease_service import AirLeaseService

        air_lease_service = AirLeaseService(
            uav_id=self.mock_drone.uav_name,
            mqtt_client=self.mock_mqtt_client
        )

        air_lease_service.send_done()

        self.mock_mqtt_client.publish.assert_not_called()

    def test_land(self):
        from dr_onboard_autonomy.air_lease_service import AirLeaseService

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
        from dr_onboard_autonomy.air_lease_service import AirLeaseService

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

