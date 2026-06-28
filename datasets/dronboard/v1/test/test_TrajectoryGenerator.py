#!/usr/bin/env python3

import math
import unittest
import time
from typing import NamedTuple
from unittest.mock import MagicMock, Mock, NonCallableMock, patch, PropertyMock

import geometry_msgs.msg
import mavros_msgs.msg
import sensor_msgs.msg
from tf.transformations import quaternion_about_axis
from droneresponse_mathtools import Lla
import numpy as np


from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    briarlla_from_lat_lon_alt,
    convert_Lla_to_LlaDict,
    convert_tuple_to_LlaDict,
)
from dr_onboard_autonomy.gimbal import Gimbal
from dr_onboard_autonomy.gimbal.data_types import Quaternion
from dr_onboard_autonomy.mavros_layer import MAVROSDrone, SetpointType
from dr_onboard_autonomy.message_senders import ReusableMessageSenders
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components import Setpoint
from dr_onboard_autonomy.states.components.trajectory import (
    _DroneData,
    _load_constraints,
    Acceleration,
    CircleTrajectory,
    Velocity,
    WaypointTrajectory,
    YawTrajectory,
    _find_angular_distance,
    _find_rotation_axis,
)

from .mock_types import mock_position_message, mock_drone

def make_mock_BaseState() -> BaseState:
    # default parameter values
    px4_params = {
        "MPC_ACC_HOR": 3.0,
        "MPC_ACC_UP_MAX": 4.0,
        "MPC_ACC_DOWN_MAX": 3.0,
        "MPC_JERK_AUTO": 4.0,
        "MPC_YAWRAUTO_MAX": 45.0,
    }

    _setpoint_type = None
    _setpoint_vec = None
    _setpoint_yaw = None

    def setpoint_prop(*args, **kwargs):
        return _setpoint_type, _setpoint_vec, _setpoint_yaw

    def send_setpoint_mock(*args, **kwargs):
        nonlocal _setpoint_type, _setpoint_vec, _setpoint_yaw
        if 'lla' in kwargs:
            _setpoint_type = SetpointType.LLA
            _setpoint_vec = kwargs['lla']
        if 'ned_position' in kwargs:
            _setpoint_type = SetpointType.NED_POSITION
            _setpoint_vec = kwargs['ned_position']
        if 'ned_velocity' in kwargs:
            _setpoint_type = SetpointType.NED_VELOCITY
            _setpoint_vec = kwargs['ned_velocity']
        if 'yaw' in kwargs and 'is_yaw_set' in kwargs:
            if kwargs['is_yaw_set']:
                _setpoint_yaw = kwargs['yaw']
    
    drone = mock_drone()
    drone.gimbal = NonCallableMock(spec=Gimbal)
    drone.gimbal.attitude = None
    drone.get_param_real = Mock()
    drone.get_param_real.side_effect = lambda param_name: px4_params[param_name]
    message_senders = NonCallableMock(spec=ReusableMessageSenders)
    mqtt_client = NonCallableMock(spec=MQTTClient)
    local_mqtt_client = NonCallableMock(spec=MQTTClient)
    kw = {"outcomes": ["human_control"], "local_mqtt_client": local_mqtt_client}
    return BaseState(drone=drone, reusable_message_senders=message_senders, mqtt_client=mqtt_client, **kw)


def mock_message(message_type, data=None):
    return {
        'type': message_type,
        'data': data
    }

def mock_state_msg(mode: str):
    return mock_message("state", NonCallableMock(spec=mavros_msgs.msg.State, mode=mode))


ImuAcceleration = NamedTuple("ImuAcceleration", x=float, y=float, z=float)
ImuQuaternion = NamedTuple("ImuQuaternion", x=float, y=float, z=float, w=float)


def mock_imu_msg(acceleration: ImuAcceleration, quaternion: ImuQuaternion):
    msg_time = time.time()
    # acceleration
    imu_msg = NonCallableMock(spec=sensor_msgs.msg.Imu)
    imu_msg.header.stamp.to_sec.side_effect = lambda: msg_time
    imu_msg.header.stamp.to_time.side_effect = lambda: msg_time
    # data arrives in ENU
    imu_props = {
        "linear_acceleration.x": acceleration.x,
        "linear_acceleration.y": acceleration.y,
        "linear_acceleration.z": acceleration.z,
        "orientation.x": quaternion.x,
        "orientation.y": quaternion.y,
        "orientation.z": quaternion.z,
        "orientation.w": quaternion.w,
    }
    imu_msg.configure_mock(**imu_props)
    return mock_message("imu", imu_msg)

def mock_velocity_msg(x: float, y: float, z: float):
    # velocity
    vel_msg = NonCallableMock(spec=geometry_msgs.msg.TwistStamped)
    # data arrives in ENU
    vel_props = {
        "twist.linear.x": x,
        "twist.linear.y": y,
        "twist.linear.z": z,
    }
    vel_msg.configure_mock(**vel_props)
    return mock_message("velocity", vel_msg)

def mock_position_msg(lat: float, lon: float, alt: float):
    return mock_position_message(lat, lon, alt)

RawVelocity = NamedTuple("RawVelocity", x=float, y=float, z=float)
def mock_all_drone_data(state: str, position: BriarLla, velocity: RawVelocity, acceleration: ImuAcceleration, attitude: ImuQuaternion):
     
    return [
        mock_state_msg(state),
        mock_imu_msg(acceleration, attitude),
        mock_velocity_msg(velocity.x, velocity.y, velocity.z),
        mock_position_msg(*position.ellipsoid.tup),    
    ]

def mock_setpoint_driver(dr_state: BaseState = None):
    if dr_state is None:
        setpoint_driver = NonCallableMock(spec=Setpoint)
    else:
        setpoint_driver = Setpoint(dr_state)
    return setpoint_driver

class TestTrajectoryGenerator(unittest.TestCase):

    def test_load_load_constraints_function(self):
        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone
        drone.get_param_real.side_effect = [1.0, 2.0, 3.0, 4.0, 5.0]

        params = _load_constraints(drone)

        self.assertEqual(params["MPC_ACC_HOR"], 1.0, "parameter value is wrong MPC_ACC_HOR")
        self.assertEqual(params["MPC_ACC_UP_MAX"], 2.0, "parameter value is wrong MPC_ACC_UP_MAX")
        self.assertEqual(params["MPC_ACC_DOWN_MAX"], 3.0, "parameter value is wrong MPC_ACC_DOWN_MAX")
        self.assertEqual(params["MPC_JERK_AUTO"], 4.0, "parameter value is wrong MPC_JERK_AUTO")
        self.assertEqual(params["MPC_YAWRAUTO_MAX"], 5.0, "parameter value is wrong MPC_YAWRAUTO_MAX")

    def test_load_load_constraints_function_with_defaults(self):
        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone
        params = _load_constraints(drone)

        self.assertEqual(params["MPC_ACC_HOR"], 3.0, "parameter value is wrong MPC_ACC_HOR")
        self.assertEqual(params["MPC_ACC_UP_MAX"], 4.0, "parameter value is wrong MPC_ACC_UP_MAX")
        self.assertEqual(params["MPC_ACC_DOWN_MAX"], 3.0, "parameter value is wrong MPC_ACC_DOWN_MAX")
        self.assertEqual(params["MPC_JERK_AUTO"], 4.0, "parameter value is wrong MPC_JERK_AUTO")


    
    def test_DroneData_can_receive_messages_and_update_fields(self):
        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone
        trajectory_data = _DroneData(dr_state)
        
        # state
        self.assertEqual(trajectory_data.mode, None)
        state_msg = mock_message("state", NonCallableMock(spec=mavros_msgs.msg.State, mode="OFFBOARD"))
        trajectory_data._on_state_message(state_msg)
        self.assertEqual(trajectory_data.mode, "OFFBOARD")
        self.assertTrue(not trajectory_data.is_data_available())

        # acceleration
        self.assertEqual(trajectory_data.acceleration, None)
        imu_msg = NonCallableMock(spec=sensor_msgs.msg.Imu)
        # data arrives in ENU
        imu_props = {
            "linear_acceleration.x": 10.0,
            "linear_acceleration.y": 20.0,
            "linear_acceleration.z": 30.0,
        }
        imu_msg.configure_mock(**imu_props)
        trajectory_data._on_imu_message(mock_message("imu", imu_msg))
        actual_a = trajectory_data.acceleration
        # we expect the data in NED
        expected_a = (20.0, 10.0, -30.0)
        self.assertEqual(actual_a, expected_a)
        self.assertTrue(not trajectory_data.is_data_available())

        # velocity
        self.assertEqual(trajectory_data.velocity, None)
        vel_msg = NonCallableMock(spec=geometry_msgs.msg.TwistStamped)
        # data arrives in ENU
        vel_props = {
            "twist.linear.x": 10.0,
            "twist.linear.y": 20.0,
            "twist.linear.z": 30.0,
        }
        vel_msg.configure_mock(**vel_props)
        trajectory_data._on_velocity_message(mock_message("velocity", vel_msg))
        actual_v = trajectory_data.velocity
        # we expect the data in NED
        expected_v = (20.0, 10.0, -30.0)
        self.assertEqual(actual_v, expected_v)
        self.assertTrue(not trajectory_data.is_data_available())

        # position
        self.assertEqual(trajectory_data.position, None)
        navSatFix_msg = NonCallableMock(spec=sensor_msgs.msg.NavSatFix, latitude=45.0, longitude=90.0, altitude=0)
        pos_msg = mock_message("position", navSatFix_msg)
        trajectory_data._on_position_message(pos_msg)
        p = trajectory_data.position
        actual_tup_wgs84 = p.ellipsoid.tup
        expected_tup_wgs84 = 45.0, 90.0, 0.0
        self.assertEqual(actual_tup_wgs84, expected_tup_wgs84)
        self.assertIsNotNone(p.amsl.tup)

        # is_data_available
        self.assertTrue(trajectory_data.is_data_available())





class TestWaypointTrajectory(unittest.TestCase):

    def test_WaypointTrajectory_init(self):
        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone
        trajectory = WaypointTrajectory(dr_state)

        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)
        self.assertEqual(type(trajectory.data), _DroneData)

    def test_that_the_handlers_are_wired_up(self):
        dr_state: BaseState = make_mock_BaseState()
        t = WaypointTrajectory(dr_state)

        drone_mode = "TEST"
        dr_state.handlers.notify(mock_state_msg(drone_mode))
        self.assertEqual(t.data.mode, drone_mode)

        # test the imu handler
        linear_acceleration = ImuAcceleration(1.0, 2.0, 3.0)
        orientation = quaternion_about_axis(math.pi / 2, (0, 0, 1)) # North = 90 degrees about the z axis in ENU
        orientation = ImuQuaternion(*orientation)
        dr_state.handlers.notify(mock_imu_msg(linear_acceleration, orientation))
        self.assertEqual(t.data.acceleration, (2.0, 1.0, -3.0))
        self.assertEqual(t.data.attitude.x, orientation.x)
        self.assertEqual(t.data.attitude.y, orientation.y)
        self.assertEqual(t.data.attitude.z, orientation.z)
        self.assertEqual(t.data.attitude.w, orientation.w)
        
        # test the velocity handler
        dr_state.handlers.notify(mock_velocity_msg(10.0, 20.0, 30.0))
        self.assertEqual(t.data.velocity, (20.0, 10.0, -30.0))

        # test the position handler
        rc_club = (41.60667852830868, -86.35530652323709, 229.0)
        rc_club = BriarLla(convert_tuple_to_LlaDict(rc_club), is_amsl=True)
        dr_state.handlers.notify(mock_position_msg(*rc_club.ellipsoid.tup))
        self.assertEqual(t.data.position.amsl.tup, rc_club.amsl.tup)
        
        update_msg = mock_message(Setpoint.TIMER_MESSAGE_NAME, 0.2)
        self.assertEqual(t._internal_state, t.State.STARTING)
        dr_state.handlers.notify(update_msg)
        self.assertEqual(t._internal_state, t.State.STARTING)

        # test the state handler (the message from the mavros/state topic)
        dr_state.handlers.notify(mock_state_msg("OFFBOARD"))
        self.assertEqual(t.data.mode, "OFFBOARD")
        dr_state.handlers.notify(update_msg)
        self.assertEqual(t._internal_state, t.State.STARTING)

        dest = rc_club.ellipsoid.lla.move_ned(10, 15, -8)
        dest = BriarLla(convert_Lla_to_LlaDict(dest), is_amsl=False)
        t.fly_to_waypoint(dest.amsl.lla, None)
        self.assertEqual(t._internal_state, t.State.STARTING)

        t.fly_to_waypoint(dest.amsl.lla, 2.0)
        self.assertEqual(t._internal_state, t.State.STARTING)

        dr_state.handlers.notify(update_msg)
        self.assertEqual(t._internal_state, t.State.MOVING)

    def test_internal_state_tx_from_starting_to_moving(self):
        """The goal is to feed the component the right sequence of messages so that
        the internal state switches from STARTING to MOVING.
        """
        dr_state: BaseState = make_mock_BaseState()
        trajectory = WaypointTrajectory(dr_state)
        self.assertTrue(trajectory._is_starting())
        trajectory._on_trajectory_update({})

        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)

        # prepare the data
        trajectory.data._mode = "OFFBOARD"
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)
        lla_tup = (0,0,0)
        trajectory.data._position = BriarLla(convert_tuple_to_LlaDict(lla_tup))
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)

        trajectory.data._velocity_ned = Velocity(0, 0, 0)
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)

        trajectory.data._acceleration_ned = Acceleration(0, 0, 0)
        trajectory._on_trajectory_update({})

        # The trajactory has everything except attitude 
        self.assertFalse(trajectory.data.is_data_available(), "data is not available when attitude is missing")
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)

        # add attitude
        trajectory.data._attitude = Quaternion(0, 0, 0, 1)
        self.assertTrue(trajectory.data.is_data_available(), "data is available when all data is present")
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)


        final_pos_wgs84: Lla = trajectory.data._position.ellipsoid.lla.move_ned(20, 10, 0.0)
        final_pos = BriarLla(convert_Lla_to_LlaDict(final_pos_wgs84), is_amsl=False)
        trajectory.fly_to_waypoint(final_pos.amsl.lla, None)
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)

        trajectory.fly_to_waypoint(final_pos.amsl.lla, 5.0)
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.MOVING)

    @patch('dr_onboard_autonomy.states.components.trajectory.time', spec=True)
    def test_internal_state_tx_from_moving_to_done(self, mock_time: MagicMock):
        """
        """
        time_now = 1659574032.3075
        count = 0
        
        def next_time():
            nonlocal count
            result = time_now + count * 0.2
            count = count + 1
            return result

        mock_time.time.side_effect = next_time

        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone

        initial_pos = 41.606685488280505, -86.35537617203288, 240.0
        initial_pos = convert_tuple_to_LlaDict(initial_pos)
        initial_pos = BriarLla(initial_pos)

        final_pos = initial_pos.ellipsoid.lla.move_ned(-30, -20, -1.0)
        final_pos = convert_Lla_to_LlaDict(final_pos)
        final_pos = BriarLla(final_pos, is_amsl=False)

        initial_vel = (0.1, 0.01, -0.002)
        initial_acceleration = (0.0007, 0.00001, -0.0002)
        initial_attitude = Quaternion(0, 0, 0, 1)

        trajectory_comp = WaypointTrajectory(dr_state)

        trajectory_comp.data._mode = "OFFBOARD"
        trajectory_comp.data._acceleration_ned = initial_acceleration
        trajectory_comp.data._velocity_ned = initial_vel
        trajectory_comp.data._position = initial_pos
        trajectory_comp.data._attitude = initial_attitude

        self.assertEqual(trajectory_comp._internal_state, trajectory_comp.State.STARTING)

        trajectory_comp.fly_to_waypoint(final_pos.amsl.lla, 5.59)
        self.assertEqual(trajectory_comp._internal_state, trajectory_comp.State.STARTING)

        self.assertIsNone(trajectory_comp.setpoint_driver.lla)
        
        trajectory_comp._on_trajectory_update({})

        self.assertIsNotNone(trajectory_comp.setpoint_driver.lla)
        # make sure the first setpoint is near the starting position 
        for actual, expected in zip(trajectory_comp.setpoint_driver.lla, initial_pos.amsl.tup):
            self.assertAlmostEqual(actual, expected)
        
        while not trajectory_comp.is_done():
            self.assertEqual(trajectory_comp._internal_state, trajectory_comp.State.MOVING)
            last_pos = trajectory_comp.setpoint_driver.lla
            last_pos = BriarLla(convert_tuple_to_LlaDict(last_pos))
            last_dist = last_pos.ellipsoid.lla.distance(final_pos.ellipsoid.lla)
            
            trajectory_comp._on_trajectory_update({})
            current_pos = trajectory_comp.setpoint_driver.lla
            current_pos = BriarLla(convert_tuple_to_LlaDict(current_pos))
            current_distance = current_pos.ellipsoid.lla.distance(final_pos.ellipsoid.lla)

            # if we're flying in a straight line from the initial position to the final position
            # then every setpoint should be a little closer than the last...
            # Unless we're at the end of our trajectory.
            # In that case, it's ok for the distance to be zero (plus some numerical accuracy).
            self.assertTrue(current_distance < last_dist or current_distance < 1e-7 )
        
        self.assertEqual(trajectory_comp._internal_state, trajectory_comp.State.DONE)
    

class TestCircleTrajectory(unittest.TestCase):
    def test_circle_trajectory_init(self):
        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone

        circle_trajectory = CircleTrajectory(dr_state)
        self.assertEqual(circle_trajectory._internal_state, circle_trajectory.State.STARTING)

    def test_circle_trajectory_data_handlers(self):
        dr_state: BaseState = make_mock_BaseState()
        rc_club = briarlla_from_lat_lon_alt(41.60667852830868, -86.35530652323709, 229.0, is_amsl=True)
        sensor_messages = mock_all_drone_data(
            state="AUTO.LOITER",
            position=rc_club,
            velocity=RawVelocity(0.01, 0.02, 0.03),
            acceleration=ImuAcceleration(1.0, 2.0, 3.0),
            attitude=ImuQuaternion(0.0, 0.0, 0.0, 1.0),
        )
        circle_trajectory = CircleTrajectory(dr_state)

        self.assertEqual(circle_trajectory._internal_state, circle_trajectory.State.STARTING)
        self.assertFalse(circle_trajectory.data.is_data_available(), "The data should not be available yet")
        self.assertTrue(circle_trajectory._is_starting(), "The trajectory should be starting")
        for msg in sensor_messages:
            dr_state.handlers.notify(msg)
            self.assertTrue(circle_trajectory._is_starting())
        
        self.assertEqual(circle_trajectory._internal_state, circle_trajectory.State.STARTING)
        self.assertTrue(circle_trajectory.data.is_data_available(), "The data should be available now")

        # state message
        self.assertEqual(circle_trajectory.data.mode, "AUTO.LOITER")

        # position message
        self.assertEqual(circle_trajectory.data.position.amsl.lla.latitude, rc_club.amsl.lla.latitude)
        self.assertEqual(circle_trajectory.data.position.amsl.lla.longitude, rc_club.amsl.lla.longitude)
        self.assertEqual(circle_trajectory.data.position.amsl.lla.altitude, rc_club.amsl.lla.altitude)
        self.assertEqual(circle_trajectory.data.position.ellipsoid.lla.latitude, rc_club.ellipsoid.lla.latitude)
        self.assertEqual(circle_trajectory.data.position.ellipsoid.lla.longitude, rc_club.ellipsoid.lla.longitude)
        self.assertEqual(circle_trajectory.data.position.ellipsoid.lla.altitude, rc_club.ellipsoid.lla.altitude)

        # velocity message
        self.assertEqual(circle_trajectory.data.velocity, Velocity(north=0.02, east=0.01, down=-0.03))

        # imu message    
        self.assertEqual(circle_trajectory.data.acceleration, (2.0, 1.0, -3.0))
        self.assertEqual(circle_trajectory.data.attitude, Quaternion(0.0, 0.0, 0.0, 1.0))

    def test_circle_trajectory_tx_starting_to_moving(self):
        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone

        circle_trajectory = CircleTrajectory(dr_state)

        self.assertTrue(circle_trajectory._is_starting())
        self.assertEqual(circle_trajectory._internal_state, circle_trajectory.State.STARTING)

        rc_club = briarlla_from_lat_lon_alt(41.60667852830868, -86.35530652323709, 229.0, is_amsl=True)
        drone_pos = rc_club.amsl.lla.move_ned(0.0, 10.0, -10.0)
        drone_pos = briarlla_from_lat_lon_alt(drone_pos.latitude, drone_pos.longitude, drone_pos.altitude, is_amsl=True)
        sensor_messages = mock_all_drone_data(
            state="OFFBOARD",
            position=drone_pos,
            velocity=RawVelocity(0.01, 0.02, 0.03),
            acceleration=ImuAcceleration(1.0, 2.0, 3.0),
            attitude=ImuQuaternion(0.0, 0.0, 0.0, 1.0),
        )
        for msg in sensor_messages:
            dr_state.handlers.notify(msg)
            self.assertTrue(circle_trajectory._is_starting())
            self.assertEqual(circle_trajectory._internal_state, circle_trajectory.State.STARTING)

        self.assertEqual(circle_trajectory._internal_state, circle_trajectory.State.STARTING)
        update_msg = mock_message(Setpoint.TIMER_MESSAGE_NAME, 0.2)
        dr_state.handlers.notify(update_msg)
        self.assertEqual(circle_trajectory._internal_state, circle_trajectory.State.STARTING)

        circle_trajectory.fly_circle(rc_club.amsl.lla, 180.0, 2.0)
        self.assertEqual(circle_trajectory._center_position.ellipsoid.lla, rc_club.ellipsoid.lla)
        self.assertEqual(circle_trajectory._sweep_angle, 180.0)
        self.assertEqual(circle_trajectory._speed_limit, 2.0)

        dr_state.handlers.notify(update_msg)
        self.assertEqual(circle_trajectory._internal_state, circle_trajectory.State.MOVING)

    @patch('dr_onboard_autonomy.states.components.trajectory.time', spec=True)
    def test_circle_trajectory_setpoints(self, mock_time: MagicMock):
        time_now = 1659574032.3075
        count = 0
        
        def next_time():
            nonlocal count
            result = time_now + count * 0.2
            count = count + 1
            return result

        mock_time.time.side_effect = next_time

        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone

        circle_trajectory = CircleTrajectory(dr_state)

        rc_club = briarlla_from_lat_lon_alt(41.60667852830868, -86.35530652323709, 229.0, is_amsl=True)
        drone_pos = rc_club.amsl.lla.move_ned(0.0, 10.0, -10.0)
        drone_pos = briarlla_from_lat_lon_alt(drone_pos.latitude, drone_pos.longitude, drone_pos.altitude, is_amsl=True)
        sensor_messages = mock_all_drone_data(
            state="OFFBOARD",
            position=drone_pos,
            velocity=RawVelocity(0.01, 0.02, 0.03),
            acceleration=ImuAcceleration(1.0, 2.0, 3.0),
            attitude=ImuQuaternion(0.0, 0.0, 0.0, 1.0),
        )
        for msg in sensor_messages:
            dr_state.handlers.notify(msg)

        circle_trajectory.fly_circle(rc_club.amsl.lla, 180.0, 2.0)

        update_msg = mock_message(Setpoint.TIMER_MESSAGE_NAME, 0.2)
        dr_state.handlers.notify(update_msg)

        self.assertEqual(circle_trajectory._internal_state, circle_trajectory.State.MOVING)
        # now that we're moving, we should be able to get the setpoints

        # the initial angle is pi/2 radians (90 degrees) because we started at 10 meters east of the center
        # the angle is measured around the down axis
        # where an angle of 0 is north, pi/2 radians is east, pi radians is south, and 3*pi/2 radians is west
        # we should sweep from 90 degrees to 270 degrees (pi/2 radians to 3*pi/2 radians)
        initial_angle = math.pi / 2
        # the amount of angular sweep is zero at the start
        last_angle = 0.0
        while not circle_trajectory.is_done():
            dr_state.handlers.notify(update_msg)

            # the trajectory outputs a position by assigning one to the setpoint driver
            actual = circle_trajectory.setpoint_driver.lla
            actual = briarlla_from_lat_lon_alt(*actual, is_amsl=True)
            
            ## find the angle of the drone around the circle
            n, e, d = rc_club.ellipsoid.lla.distance_ned(actual.ellipsoid.lla)
            angle = math.atan2(e, n)
            
            if angle < 0:
                angle = angle + 2 * math.pi
            # angle is the amount of angular sweep so far
            # the angle should increase from 0 to 180 degrees (0 to pi radians)
            angle = angle - initial_angle

            # the angle should be increasing until it reaches the final angle
            # then it should stay there forever.
            # the done method waits TRANSITION_DELAY extra seconds after reaching the final angle
            # so we should see the angle not change dispite the is_done method returning false
            if abs(angle - math.pi) > 1e-9:
                self.assertGreater(angle, last_angle, "the angle should grow until the trajectory is done")

            last_angle = angle
        
        # at the end of the trajectory, we should see the sweep angle (180 degrees) but in radians (pi)
        self.assertAlmostEqual(last_angle, math.pi, delta=1e-3)


class TestYawTrajectory(unittest.TestCase):
    def test_init_method(self):
        dr_state: BaseState = make_mock_BaseState()
        yaw_trajectory = YawTrajectory(dr_state)
        self.assertTrue(yaw_trajectory._this_owns_setpoint_driver)

        setpoint_driver = mock_setpoint_driver(dr_state)
        yaw_trajectory = YawTrajectory(dr_state, setpoint_driver)
        self.assertTrue(not yaw_trajectory._this_owns_setpoint_driver)

    
    def test_angular_distance(self):
        yaw0 = (-1.0 / 2.0) * math.pi
        yaw1 = (3.0 / 4.0) * math.pi

        # should be 135 degrees
        self.assertAlmostEqual(_find_angular_distance(yaw0, yaw1), 135.0 * math.pi / 180.0, delta=1e-9)

    @patch("dr_onboard_autonomy.states.components.trajectory.np")
    def test_angular_distance_float_driven_domain_breach(self, mock_np):
        yaw0 = 0.03892402794757732
        yaw1 = 0.03892402438199675
        '''
        above inputs will result in a value provided to math.acos of just over 1.0 due to 
        floating point error. mocking numpy to force this behavior for both > 1.0 and < -1.0
        '''

        dot_products = (
            {
                "np_dot_prod": 1.0000000000000002,
                "expected_angular_distance": 0.0
            },
            {
                "np_dot_prod": -1.0000000000000002,
                "expected_angular_distance": math.pi
            }
        )

        for test in dot_products:
            with self.subTest(f"testing dot prod result {test['np_dot_prod']}"):
                mock_np.reset_mock()
                mock_np.dot.return_value = test["np_dot_prod"]

                self.assertEqual(
                    _find_angular_distance(yaw0, yaw1),
                    test["expected_angular_distance"]
                )
    
    def test_find_rotation_axis(self):
        yaw0 = (-1.0 / 2.0) * math.pi
        yaw1 = (3.0 / 4.0) * math.pi

        # this should be around the down axis
        axis = _find_rotation_axis(yaw0, yaw1)

        self.assertEqual(axis, -1)

        axis = _find_rotation_axis(yaw1, yaw0)
        self.assertEqual(axis, 1)

        axis = _find_rotation_axis(yaw0, yaw0)
        self.assertEqual(axis, 1)

        axis = _find_rotation_axis(yaw1, yaw1)
        self.assertEqual(axis, 1)

        # these angles are the same
        # this should trigger the "same direction" case
        axis = _find_rotation_axis(-math.pi, math.pi)
        self.assertEqual(axis, 1)

    @patch('dr_onboard_autonomy.states.components.trajectory.time', spec=True)
    def test_turn_counter_clockwise_156_degrees(self, mock_time: MagicMock):
        """ this tests the yaw trajectory that turns to look at the shed when flying
        the takeoff mission at peppermint road.
        """

        time_initial = 1659574032.3075
        count = 0
        
        def next_time():
            nonlocal count
            result = time_initial + count * 0.02
            count = count + 1
            return result

        mock_time.time.side_effect = next_time

        dr_state: BaseState = make_mock_BaseState()

        def send_update():
            nonlocal dr_state
            dr_state.handlers.notify(mock_message(Setpoint.TIMER_MESSAGE_NAME, 0.02))
        
        drone = dr_state.drone

        yaw_trajectory = YawTrajectory(dr_state)
        
        sensor_messages = mock_all_drone_data(
            state="OFFBOARD",
            position=briarlla_from_lat_lon_alt(41.60667852830868, -86.35530652323709, 229.0, is_amsl=True),
            velocity=RawVelocity(0.0, 0.0, 0.0),
            acceleration=ImuAcceleration(0.0, 0.0, 0.0),
            attitude=ImuQuaternion(0.0, 0.0, 0.0, 1.0), # yaw = 0 (facing east)
        )
        for msg in sensor_messages:
            self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.STARTING)
            dr_state.handlers.notify(msg)
            send_update()

        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.HOLDING)
        yaw_trajectory.yaw = 2.7230879688543905
        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.STARTING)
        send_update()
        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.MOVING)

        # make sure all the fields are set correctly
        assert yaw_trajectory._yaw_target == 2.7230879688543905
        assert yaw_trajectory._yaw_initial == 0.0
        assert yaw_trajectory._yaw_delta == 2.7230879688543905
        assert yaw_trajectory._spin_axis == 1
        assert yaw_trajectory._t_start == time_initial
        assert yaw_trajectory.setpoint_driver.yaw == 0.0

        # the angular distance should be decreasing
        last_angular_distance = yaw_trajectory._yaw_delta
        while yaw_trajectory._internal_state == yaw_trajectory.State.MOVING:
            send_update()
            yaw_setpoint = yaw_trajectory.setpoint_driver.yaw
            angular_distance = _find_angular_distance(yaw_setpoint, yaw_trajectory._yaw_target)
            assert angular_distance < last_angular_distance or angular_distance < 1e-6
            last_angular_distance = angular_distance

        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.HOLDING)
        send_update()
        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.HOLDING)

        drone_yaw = 2.7230879688543905
        drone_attitude = quaternion_about_axis(drone_yaw, [0, 0, 1])
        imu_msg = mock_imu_msg(ImuAcceleration(0.0, 0.0, 0.0), ImuQuaternion(*drone_attitude))
        dr_state.handlers.notify(imu_msg)
        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.HOLDING)

        # now lets change the yaw setpoint only a little
        yaw_target = drone_yaw - 0.5
        yaw_trajectory.yaw = yaw_target
        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.STARTING)
        send_update()
        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.MOVING)
        assert yaw_trajectory._spin_axis == -1

        self.assertAlmostEqual(yaw_trajectory._spin_axis, -1)
        self.assertAlmostEqual(yaw_trajectory._yaw_delta, 0.5)

        # the angular distance should be decreasing
        last_angular_distance = yaw_trajectory._yaw_delta
        while yaw_trajectory._internal_state == yaw_trajectory.State.MOVING:
            send_update()
            yaw_setpoint = yaw_trajectory.setpoint_driver.yaw
            angular_distance = _find_angular_distance(yaw_setpoint, yaw_trajectory._yaw_target)
            x = angular_distance < last_angular_distance or angular_distance < 1e-6
            if not x:
                assert x
            last_angular_distance = angular_distance
        
        
    
    @unittest.skip("???")
    @patch('dr_onboard_autonomy.states.components.trajectory.time', spec=True)
    def test_inital_yaw_uses_last_yaw_setpoint(self, mock_time: MagicMock):
        """ this tests the yaw trajectory's starting yaw is the previous yaw setpoint
        when the previous yaw is within the threshold angle of the sensed yaw.
        """

        time_initial = 1659574032.3075
        count = 0
        
        def next_time():
            nonlocal count
            result = time_initial + count * 0.02
            count = count + 1
            return result

        mock_time.time.side_effect = next_time

        dr_state: BaseState = make_mock_BaseState()

        def send_update():
            nonlocal dr_state
            dr_state.handlers.notify(mock_message(Setpoint.TIMER_MESSAGE_NAME, 0.02))
        
        drone = dr_state.drone
        yaw_setpoint = YawTrajectory.THRESHOLD_ANGLE - math.radians(0.01)
        drone.send_setpoint(lla=(0,0,0), yaw=yaw_setpoint, is_yaw_set=True)

        yaw_trajectory = YawTrajectory(dr_state)
        
        sensor_messages = mock_all_drone_data(
            state="OFFBOARD",
            position=briarlla_from_lat_lon_alt(41.60667852830868, -86.35530652323709, 229.0, is_amsl=True),
            velocity=RawVelocity(0.0, 0.0, 0.0),
            acceleration=ImuAcceleration(0.0, 0.0, 0.0),
            attitude=ImuQuaternion(0.0, 0.0, 0.0, 1.0), # yaw = 0 (facing east)
        )
        for msg in sensor_messages:
            self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.STARTING)
            dr_state.handlers.notify(msg)
            send_update()

        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.HOLDING)
        yaw_trajectory.yaw = 2.7230879688543905
        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.STARTING)
        send_update()
        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.MOVING)

        # make sure all the fields are set correctly
        assert yaw_trajectory._yaw_target == 2.7230879688543905
        assert yaw_trajectory._yaw_initial == yaw_setpoint
        assert yaw_trajectory._yaw_delta == yaw_trajectory._yaw_target - yaw_setpoint


    @patch('dr_onboard_autonomy.states.components.trajectory.time', spec=True)
    def test_inital_yaw_uses_sensed_yaw(self, mock_time: MagicMock):
        """ this tests the yaw trajectory's starting yaw is the IMU yaw
        when the previous yaw setpoint is not within threshold angle of the sensed yaw.
        """

        time_initial = 1659574032.3075
        count = 0
        
        def next_time():
            nonlocal count
            result = time_initial + count * 0.02
            count = count + 1
            return result

        mock_time.time.side_effect = next_time

        dr_state: BaseState = make_mock_BaseState()

        def send_update():
            nonlocal dr_state
            dr_state.handlers.notify(mock_message(Setpoint.TIMER_MESSAGE_NAME, 0.02))
        
        drone = dr_state.drone
        yaw_setpoint = YawTrajectory.THRESHOLD_ANGLE + math.radians(0.01)
        drone.send_setpoint(lla=(0,0,0), yaw=yaw_setpoint, is_yaw_set=True)

        yaw_trajectory = YawTrajectory(dr_state)
        
        sensor_messages = mock_all_drone_data(
            state="OFFBOARD",
            position=briarlla_from_lat_lon_alt(41.60667852830868, -86.35530652323709, 229.0, is_amsl=True),
            velocity=RawVelocity(0.0, 0.0, 0.0),
            acceleration=ImuAcceleration(0.0, 0.0, 0.0),
            attitude=ImuQuaternion(0.0, 0.0, 0.0, 1.0), # yaw = 0 (facing east)
        )
        for msg in sensor_messages:
            self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.STARTING)
            dr_state.handlers.notify(msg)
            send_update()

        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.HOLDING)
        yaw_trajectory.yaw = 2.7230879688543905
        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.STARTING)
        send_update()
        self.assertEqual(yaw_trajectory._internal_state, yaw_trajectory.State.MOVING)

        # make sure all the fields are set correctly
        assert yaw_trajectory._yaw_target == 2.7230879688543905
        assert yaw_trajectory._yaw_initial == 0.0
        assert yaw_trajectory._yaw_delta == yaw_trajectory._yaw_target
        # the angular distance is from zero to 2.7230879688543905 
        # last_angle = 0.0
        # while not yaw_trajectory.is_done():
        #     dr_state.handlers.notify(update_msg)

        #     # the trajectory outputs a position by assigning one to the setpoint driver
        #     actual = yaw_trajectory.setpoint_driver.yaw

        #     # the angle should be increasing until it reaches the final angle
        #     # then it should stay there forever.
        #     # the done method waits TRANSITION_DELAY extra seconds after reaching the final angle
        #     # so we should see the angle not change dispite the is_done method returning false
        #     if abs(actual - math.pi) > 1e-9:
        #         self.assertGreater(actual, last_angle, "the angle should grow until the trajectory is done")

        #     last_angle = actual
        
        # at the end of the trajectory, we should see the sweep angle (180 degrees) but in radians (pi)