#!/usr/bin/env python3

import math
import queue
import unittest
from typing import NamedTuple
from unittest.mock import MagicMock, NonCallableMock, patch

import numpy as np
from tf.transformations import quaternion_about_axis
from droneresponse_mathtools import Lla

from dr_onboard_autonomy.briar_helpers import (
    BriarLla,
    briarlla_from_lat_lon_alt,
    convert_Lla_to_LlaDict,
    convert_tuple_to_LlaDict,
)
from dr_onboard_autonomy.gimbal.data_types import Quaternion
from dr_onboard_autonomy.mavros_layer import SetpointType
from dr_onboard_autonomy.message_senders import ReusableMessageSenders
from dr_onboard_autonomy.models import (
    CopterParameters,
    DATUM_REFERENCE,
    FCUCopterMode,
    FCUState,
    FCUStatus,
    IMU,
    LlaPosition,
    NedAcceleration,
    NedVelocity
)
from dr_onboard_autonomy.models import Quaternion as QuaternionModel
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
    TRANSITION_DELAY,
)

from . import mock_types
from .mock_types import mock_copter_drone

def make_mock_BaseState() -> BaseState:
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

    drone = mock_copter_drone()
    drone._load_parameters.return_value = CopterParameters(
        takeoff_altitude=7.0,
        horizontal_acceleration_limit=3.0,
        upward_acceleration_limit=4.0,
        downward_acceleration_limit=3.0,
        jerk_limit=4.0,
        maximum_yaw_rate=45.0
    )
    # drone.get_fcu_parameter.side_effect = lambda param_name: px4_params[param_name]
    message_senders = NonCallableMock(spec=ReusableMessageSenders)
    mqtt_client = NonCallableMock(spec=MQTTClient)
    local_mqtt_client = NonCallableMock(spec=MQTTClient)
    kw = {"outcomes": ["failsafe"], "local_mqtt_client": local_mqtt_client}
    return BaseState(drone=drone, reusable_message_senders=message_senders, mqtt_client=mqtt_client, **kw)


def mock_message(message_type, data=None):
    return {
        'type': message_type,
        'data': data
    }


def mock_state_msg(mode: FCUCopterMode):
    return mock_message("state", FCUState(
        armed=True,
        connected=True,
        mode=mode,
        status=FCUStatus.ACTIVE
    ))


ImuAcceleration = NamedTuple("ImuAcceleration", x=float, y=float, z=float)
ImuQuaternion = NamedTuple("ImuQuaternion", x=float, y=float, z=float, w=float)


def mock_imu_msg(acceleration: ImuAcceleration, quaternion: ImuQuaternion):
    # msg_time = time.time()
    # acceleration
    imu_msg = IMU(
        time = 1719504118.6429088,
        acceleration_linear = NedAcceleration(
            east=acceleration.x,
            north=acceleration.y,
            down=acceleration.z * -1.0
        ),
        attitude=QuaternionModel(
            x=quaternion.x,
            y=quaternion.y,
            z=quaternion.z,
            w=quaternion.w
        )
    )

    return mock_message("imu", imu_msg)


def mock_velocity_msg(x: float, y: float, z: float):
    # velocity data arrives in ENU from ros
    vel_msg = NedVelocity(
        east=x,
        north=y,
        down=z * -1.0
    )

    return mock_message("velocity", vel_msg)


def mock_position_msg(lat: float, lon: float, alt: float):
    return mock_message("position",
    LlaPosition(
        latitude=lat,
        longitude=lon,
        altitude=alt,
        datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
    ))


RawVelocity = NamedTuple("RawVelocity", x=float, y=float, z=float)
def mock_all_drone_data(
    mode: FCUCopterMode,
    position: BriarLla,
    velocity: RawVelocity,
    acceleration: ImuAcceleration,
    attitude: ImuQuaternion
):

    return [
        mock_state_msg(mode),
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


def create_mock_time_monotonic(w, dt):
    t = w
    def monotonic():
        nonlocal t
        result = t
        t += dt
        return result
    return monotonic


class TestTrajectoryGenerator(unittest.TestCase):

    def test_load_load_constraints_function(self):
        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone
        drone.get_fcu_parameter.side_effect = [1.0, 2.0, 3.0, 4.0, 5.0]
        drone._load_parameters.return_value = CopterParameters(
            takeoff_altitude=0.0,
            horizontal_acceleration_limit=1.0,
            upward_acceleration_limit=2.0,
            downward_acceleration_limit=3.0,
            jerk_limit=4.0,
            maximum_yaw_rate=5.0
        )

        params = _load_constraints(drone)

        self.assertEqual(
            params.horizontal_acceleration_limit,
            1.0,
            "parameter value is wrong MPC_ACC_HOR"
        )
        self.assertEqual(
            params.upward_acceleration_limit,
            2.0,
            "parameter value is wrong MPC_ACC_UP_MAX"
        )
        self.assertEqual(
            params.downward_acceleration_limit,
            3.0,
            "parameter value is wrong MPC_ACC_DOWN_MAX"
        )
        self.assertEqual(params.jerk_limit, 4.0, "parameter value is wrong MPC_JERK_AUTO")
        self.assertEqual(params.maximum_yaw_rate, 5.0, "parameter value is wrong MPC_YAWRAUTO_MAX")

    def test_load_load_constraints_function_with_defaults(self):
        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone
        params = _load_constraints(drone)

        self.assertEqual(
            params.horizontal_acceleration_limit,
            3.0,
            "parameter value is wrong MPC_ACC_HOR"
        )
        self.assertEqual(
            params.upward_acceleration_limit,
            4.0,
            "parameter value is wrong MPC_ACC_UP_MAX"
        )
        self.assertEqual(
            params.downward_acceleration_limit,
            3.0,
            "parameter value is wrong MPC_ACC_DOWN_MAX"
        )
        self.assertEqual(params.jerk_limit, 4.0, "parameter value is wrong MPC_JERK_AUTO")


    
    def test_DroneData_can_receive_messages_and_update_fields(self):
        dr_state: BaseState = make_mock_BaseState()
        drone = dr_state.drone
        trajectory_data = _DroneData(dr_state)


        # state
        self.assertEqual(trajectory_data.mode, None)
        state_msg = mock_state_msg(mode=FCUCopterMode.OFFBOARD)
        trajectory_data._on_state_message(state_msg)
        self.assertEqual(trajectory_data.mode, FCUCopterMode.OFFBOARD)
        self.assertTrue(not trajectory_data.is_data_available())

        # acceleration
        self.assertEqual(trajectory_data.acceleration, None)
        imu_msg = IMU(
            time = 1719504118.6429088,
            acceleration_linear=NedAcceleration(
                north=20.0,
                east=10.0,
                down=30.0 * -1.0
            ),
            attitude=QuaternionModel(
                x=1,
                y=0,
                z=0,
                w=1
            )
        )
        trajectory_data._on_imu_message(mock_message("imu", imu_msg))
        actual_a = trajectory_data.acceleration
        # we expect the data in NED
        expected_a = (20.0, 10.0, -30.0)
        self.assertEqual(actual_a, expected_a)
        actual_q = trajectory_data.attitude.astuple()
        expected_q = (1, 0, 0, 1)
        self.assertEqual(actual_q, expected_q)
        self.assertTrue(not trajectory_data.is_data_available())

        # velocity
        self.assertEqual(trajectory_data.velocity, None)
        vel_msg = NedVelocity(
            north= 20.0,
            east= 10.0,
            down= 30.0 * -1.0
        )
        trajectory_data._on_velocity_message(mock_message("velocity", vel_msg))
        actual_v = trajectory_data.velocity
        # we expect the data in NED
        expected_v = (20.0, 10.0, -30.0)
        self.assertEqual(actual_v, expected_v)
        self.assertTrue(not trajectory_data.is_data_available())

        # position
        self.assertEqual(trajectory_data.position, None)
        position_msg = LlaPosition(
            latitude=45.0,
            longitude=90.0,
            altitude=0.0,
            datum_ref=DATUM_REFERENCE.ELLIPSOID_WGS84
        )
        pos_msg = mock_message("position", position_msg)
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

        drone_mode = FCUCopterMode.UNKNOWN
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
        dr_state.handlers.notify(mock_state_msg(FCUCopterMode.OFFBOARD))
        self.assertEqual(t.data.mode, FCUCopterMode.OFFBOARD)
        # Note at this point we've provided all sensor data needed for a trajectory
        # and we've set the mode to OFFBOARD so the WaypointTrajectory should be ready to go
        # For these reasons we must exit the STARTING state.
        #
        # The WaypointTrajectory has _not_ been given command yet
        # so we should exit the STARTING state and enter the HOLDING state
        dr_state.handlers.notify(update_msg)
        self.assertEqual(t._internal_state, t.State.HOLDING)

        dest = rc_club.ellipsoid.lla.move_ned(10, 15, -8)
        dest = BriarLla(convert_Lla_to_LlaDict(dest), is_amsl=False)
        
        t.fly_to_waypoint(dest.llaPosition, 1.0)

        # now that we've given a command, we should be in the STARTING state
        # we will stay in the starting state until we get a timer message
        self.assertEqual(t._internal_state, t.State.STARTING)

        t.fly_to_waypoint(dest.llaPosition, 2.0)
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
        trajectory.data._mode = FCUCopterMode.OFFBOARD
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)
        lla_tup = (0,0,0)
        trajectory.data._position = BriarLla(convert_tuple_to_LlaDict(lla_tup))
        trajectory.data._pos = mock_position_msg(*lla_tup)["data"]
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)

        trajectory.data._velocity_ned = Velocity(0, 0, 0)
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)

        trajectory.data._acceleration_ned = Acceleration(0, 0, 0)
        trajectory._on_trajectory_update({})

        # The trajactory has everything except attitude 
        self.assertFalse(
            trajectory.data.is_data_available(),
            "data is not available when attitude is missing"
        )
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)

        # add attitude
        trajectory.data._attitude = Quaternion(0, 0, 0, 1)
        self.assertTrue(
            trajectory.data.is_data_available(),
            "data is available when all data is present"
        )
        trajectory._on_trajectory_update({})
        # Now the internal state should be HOLDING
        self.assertEqual(trajectory._internal_state, trajectory.State.HOLDING)


        final_pos_wgs84: Lla = trajectory.data.position.ellipsoid.lla.move_ned(20, 10, 0.0)
        final_pos = BriarLla(convert_Lla_to_LlaDict(final_pos_wgs84), is_amsl=False)
        
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.HOLDING)

        trajectory.fly_to_waypoint(final_pos.llaPosition, 5.0)
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)
        trajectory._on_trajectory_update({})
        self.assertEqual(trajectory._internal_state, trajectory.State.MOVING)

    @patch('dr_onboard_autonomy.states.components.trajectory.time', spec=True)
    def test_internal_state_tx_from_moving_to_done(self, mock_time: MagicMock):
        """
        """
        time_now = 75928.343945579

        mock_time.monotonic.side_effect = create_mock_time_monotonic(time_now, 0.00001)

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

        trajectory_comp.data._mode = FCUCopterMode.OFFBOARD
        trajectory_comp.data._acceleration_ned = initial_acceleration
        trajectory_comp.data._velocity_ned = initial_vel

        trajectory_comp.data._on_position_message(
            mock_types.mock_message_sender_position_message(*initial_pos.ellipsoid.tup)
        )
        trajectory_comp.data._attitude = initial_attitude

        self.assertEqual(trajectory_comp._internal_state, trajectory_comp.State.STARTING)

        trajectory_comp.fly_to_waypoint(final_pos.llaPosition, 5.59)
        self.assertEqual(trajectory_comp._internal_state, trajectory_comp.State.STARTING)

        self.assertIsNone(trajectory_comp.setpoint_driver.lla)

        trajectory_comp._on_trajectory_update({})

        self.assertIsNotNone(trajectory_comp.setpoint_driver.lla)
        # make sure the first setpoint is near the starting position 
        actual_lla = BriarLla.from_args(
            trajectory_comp.setpoint_driver.lla.latitude,
            trajectory_comp.setpoint_driver.lla.longitude,
            trajectory_comp.setpoint_driver.lla.altitude,
            is_amsl=True
        )

        self.assertLess(actual_lla.ellipsoid.lla.distance(initial_pos.ellipsoid.lla), 1e-3)
        for actual, expected in zip(
                (
                    trajectory_comp.setpoint_driver.lla.latitude,
                    trajectory_comp.setpoint_driver.lla.longitude,
                    trajectory_comp.setpoint_driver.lla.altitude,
                ),
                initial_pos.amsl.tup
            ):
                self.assertAlmostEqual(actual, expected)

        t_now = mock_time.monotonic()
        mock_time.monotonic.side_effect = create_mock_time_monotonic(t_now, 0.2)

        while not trajectory_comp.is_done():
            self.assertEqual(trajectory_comp._internal_state, trajectory_comp.State.MOVING)
            last_pos = trajectory_comp.setpoint_driver.lla
            last_pos = BriarLla(convert_tuple_to_LlaDict((
                last_pos.latitude,
                last_pos.longitude,
                last_pos.altitude,
            )))
            last_dist = last_pos.ellipsoid.lla.distance(final_pos.ellipsoid.lla)
            
            trajectory_comp._on_trajectory_update({})
            current_pos = trajectory_comp.setpoint_driver.lla
            current_pos = BriarLla(convert_tuple_to_LlaDict((
                current_pos.latitude,
                current_pos.longitude,
                current_pos.altitude,
            )))
            current_distance = current_pos.ellipsoid.lla.distance(final_pos.ellipsoid.lla)

            # if we're flying in a straight line from the initial position to the final position
            # then every setpoint should be a little closer than the last...
            # Unless we're at the end of our trajectory.
            # In that case, it's ok for the distance to be zero (plus some numerical accuracy).
            self.assertTrue(current_distance < last_dist or current_distance < 1e-7 )
        
        self.assertEqual(trajectory_comp._internal_state, trajectory_comp.State.HOLDING)
    
    @patch('dr_onboard_autonomy.states.components.trajectory.time', spec=True)
    def test_waypoint_trajectory_replace_trajectory_while_moving(self, mock_time: MagicMock):
        center = BriarLla.from_args(41.6066851047012, -86.35600002842635, 240.0, is_amsl=True)
        drone_pos = center
        dr_state: BaseState = make_mock_BaseState()
        dr_state.message_queue = queue.SimpleQueue()
        drone = dr_state.drone
        trajectory = WaypointTrajectory(dr_state)
        self.assertFalse(trajectory.data.is_data_available())
        sensor_msgs = mock_all_drone_data(
            mode=FCUCopterMode.OFFBOARD,
            position=drone_pos,
            velocity=RawVelocity(0.0, 0.0, 0.0),
            acceleration=ImuAcceleration(0.0, 0.0, 0.0),
            attitude=ImuQuaternion(0.0, 0.0, 0.0, 1.0),
        )
        for msg in sensor_msgs:
            dr_state.handlers.notify(msg)
        
        # We're going to simulate teh case where we couldn't restore the setpoint
        # from a previous state
        trajectory.setpoint_driver.velocity = NedVelocity(0, 0, 0)
        self.assertEqual(NedVelocity(0, 0, 0), trajectory.setpoint_driver.velocity)

        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)
        self.assertTrue(trajectory.data.is_data_available())
        self.assertFalse(trajectory._is_starting())
        # since we're we are still starting, we should not have provided a new setpoint
        self.assertEqual(NedVelocity(0, 0, 0), trajectory.setpoint_driver.velocity)

        setpoint_message = mock_message(Setpoint.TIMER_MESSAGE_NAME, 1.0/25.0)
        dr_state.handlers.notify(setpoint_message)

        # ok there are many changes we need to test
        # since we have enough data to start the trajectory, we should be in the HOLDING state
        # as we've not been given a command yet
        self.assertEqual(trajectory._internal_state, trajectory.State.HOLDING)
        # also we should have given an LLA setpoint
        self.assertIsNotNone(trajectory.setpoint_driver.lla)
        self.assertEqual(trajectory.setpoint_driver.lla, drone_pos.llaPosition)

        # now we will update the drone's location and trigger another setpoint message
        drone_pos2 = BriarLla.from_lla(drone_pos.ellipsoid.lla.move_ned(0, 0, -1.0), is_amsl=False)
        pos_msg2 = mock_position_msg(*drone_pos2.ellipsoid.tup)
        dr_state.handlers.notify(pos_msg2)
        dr_state.handlers.notify(setpoint_message)
        # make sure we're still holding
        self.assertEqual(trajectory._internal_state, trajectory.State.HOLDING)
        # make sure our setpoint is still the same
        self.assertEqual(trajectory.setpoint_driver.lla, drone_pos.llaPosition)

        # next we go back to our original position and trigger another setpoint message
        pos_msg3 = mock_position_msg(*drone_pos.ellipsoid.tup)
        dr_state.handlers.notify(pos_msg3)
        dr_state.handlers.notify(setpoint_message)
        # make sure we're still holding
        self.assertEqual(trajectory._internal_state, trajectory.State.HOLDING)
        # make sure our setpoint is still the same
        self.assertEqual(trajectory.setpoint_driver.lla, drone_pos.llaPosition)

        # Now we will command the drone to fly to the first waypoint
        west = BriarLla.from_lla(center.ellipsoid.lla.move_ned(0, -50, 0), is_amsl=False)

        trajectory.fly_to_waypoint(west.llaPosition, 5.0)
        # make sure we're in the STARTING state
        # This is because we've now given a "command" so we should exit
        # the HOLDING state and enter the STARTING state
        self.assertEqual(trajectory._internal_state, trajectory.State.STARTING)

        # now we will trigger another setpoint message
        # but first we need to mock the clock so that it behaves more
        # realistically. We should expect the WaypointTrajectory to check
        # the time twice. Once when creating the `Context` and once when
        # we calculate our first setpoint.
        time_now = 75928.343945579
        mock_time.reset_mock()
        mock_time.monotonic.side_effect = create_mock_time_monotonic(time_now, 0.000001)

        dr_state.handlers.notify(setpoint_message)

        # first let's make sure we checked the clock twice
        mock_time.monotonic.assert_called()
        # NOTE in the future it's likely ok to remove the next line:
        self.assertEqual(mock_time.monotonic.call_count, 2) 
        # I added this line to help me verify that the implementation is
        # doing what I expect but we don't require our
        # WaypointTrajectory to check the clock exactly twice when a
        # setpoint message causes a state transition from STARTING to
        # MOVING... It's just a side-effect of the implementation.

        # make sure we're in the MOVING state
        self.assertEqual(trajectory._internal_state, trajectory.State.MOVING)
        # make sure we provided the first setpoint
        self.assertIsNotNone(trajectory.setpoint_driver.lla)
        # make sure the first setpoint is near the starting position
        setpoint_pos = BriarLla.from_LlaPosition(trajectory.setpoint_driver.lla)
        self.assertLess(setpoint_pos.ellipsoid.lla.distance(drone_pos.ellipsoid.lla), 1e-6)

        # next let's fast-forward the clock so we're in the middle of the trajectory
        time_now = mock_time.monotonic()
        # to find the middle of the trajectory we will just access the duration
        # the exact time doesn't matter but we need to find a time that's
        # roughly halfway through the trajectory.
        # the halfway point below was selected experimentally
        halfway_point = (trajectory._context.time_duration - TRANSITION_DELAY) * 0.5
        # halfway_point += 44/25
        mock_time.monotonic.reset_mock()
        mock_time.monotonic.side_effect = create_mock_time_monotonic(time_now + halfway_point, 1e-9)

        dr_state.handlers.notify(setpoint_message)
        # make sure we're still in the MOVING state
        self.assertEqual(trajectory._internal_state, trajectory.State.MOVING)
        # make sure we provided a setpoint that's closer to the final position
        setpoint_pos = BriarLla.from_LlaPosition(trajectory.setpoint_driver.lla)

        # Make sure we're somewhere near the middle of the trajectory
        # we don't care about being exactly in the middle.
        # we just want to make sure we're more or less near the half-way point
        # since the distance from start to finish is 50 meters, we will make sure the setpoint
        # position is less than 30 meters from both positions
        self.assertLess(setpoint_pos.ellipsoid.lla.distance(center.ellipsoid.lla), 30) # this is the distance from the startting position to the setpoint
        self.assertLess(setpoint_pos.ellipsoid.lla.distance(west.ellipsoid.lla), 30) # this is the distance from the setpoint to the final position

        # NOTE: The following line is not needed if the implmentation changes and this causes a test failure:
        self.assertEqual(mock_time.monotonic.call_count, 2)
        # we call `mock_time.monotonic()`` twice:
        # 1. to see if we're done
        # 2. to calculate the next setpoint
        # However, this is an implementation detail and not a requirement of the WaypointTrajectory
        # It was added to verify that the implementation is doing what I expect.
        # feel free to remove this line if it causes a test failure.

        # next we will replace the trajectory with a new one
        north = BriarLla.from_lla(center.ellipsoid.lla.move_ned(50, 0, 0), is_amsl=False)
        trajectory.fly_to_waypoint(north.llaPosition, 5.0)

        # make sure we're still MOVING
        self.assertEqual(trajectory._internal_state, trajectory.State.MOVING)
        # make sure our _context final destination is the same as the new destination
        self.assertEqual(trajectory._context.position_final.ellipsoid.tup, north.ellipsoid.tup)

        # before we reset mock_time, let's simulate a "first" setpoint using the new trajectory
        dr_state.handlers.notify(setpoint_message)
        # make sure we're still MOVING
        self.assertEqual(trajectory._internal_state, trajectory.State.MOVING)
        # check the setpoint matches `setpoint_pos`
        setpoint_pos2 = BriarLla.from_LlaPosition(trajectory.setpoint_driver.lla)
        # since it's only been a few nanoseconds, since we started the new trajectory
        # we should expect the setpoint to be near where the last setpoint left off...
        
        self.assertLess(setpoint_pos2.ellipsoid.lla.distance(setpoint_pos.ellipsoid.lla), 1e-3) # On 2025-01-14 @ 16:30 I measured the actual distance to be 1.823089586683058e-08 but 1e-3 is a reasonable tolerance despite being much larger than the actual distance.

        # now we will fast-forward the clock to near the end of the trajectory
        t_final = trajectory._context.time_final - 0.01
        mock_time.monotonic.side_effect = create_mock_time_monotonic(t_final, 1e-6)
        mock_time.reset_mock()
        dr_state.handlers.notify(setpoint_message)
        # make sure we're still MOVING
        self.assertEqual(trajectory._internal_state, trajectory.State.MOVING)
        # make sure we provided a setpoint is basically at the final position
        setpoint_pos3 = BriarLla.from_LlaPosition(trajectory.setpoint_driver.lla)
        self.assertLess(setpoint_pos3.ellipsoid.lla.distance(north.ellipsoid.lla), 1e-3)
        self.assertEqual(mock_time.monotonic.call_count, 2)

        # now we will fast-forward the clock to past end of the trajectory
        t_final2 = trajectory._context.time_final + 0.01
        mock_time.monotonic.side_effect = create_mock_time_monotonic(t_final2, 1e-6)
        mock_time.reset_mock()

        dr_state.handlers.notify(setpoint_message)
        # make sure we're in the HOLDING state
        self.assertEqual(trajectory._internal_state, trajectory.State.HOLDING)
        # make sure the setpoint is at the final position
        setpoint_pos4 = BriarLla.from_LlaPosition(trajectory.setpoint_driver.lla)
        self.assertLess(setpoint_pos4.ellipsoid.lla.distance(north.ellipsoid.lla), 1e-3)
        self.assertEqual(mock_time.monotonic.call_count, 2)






    

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
            mode=FCUCopterMode.LOITER,
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
        self.assertEqual(circle_trajectory.data.mode, FCUCopterMode.LOITER)

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
            mode=FCUCopterMode.OFFBOARD,
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
            mode=FCUCopterMode.OFFBOARD,
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
            actual = (
                circle_trajectory.setpoint_driver.lla.latitude,
                circle_trajectory.setpoint_driver.lla.longitude,
                circle_trajectory.setpoint_driver.lla.altitude,
            )
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

        mock_time.monotonic.side_effect = next_time

        dr_state: BaseState = make_mock_BaseState()

        def send_update():
            nonlocal dr_state
            dr_state.handlers.notify(mock_message(Setpoint.TIMER_MESSAGE_NAME, 0.02))

        drone = dr_state.drone

        yaw_trajectory = YawTrajectory(dr_state)

        sensor_messages = mock_all_drone_data(
            mode=FCUCopterMode.OFFBOARD,
            position=briarlla_from_lat_lon_alt(
                41.60667852830868,
                -86.35530652323709,
                229.0,
                is_amsl=True
            ),
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
        assert yaw_trajectory._trajectory_data.yaw_initial == 0.0
        assert yaw_trajectory._trajectory_data.angular_displacement == 2.7230879688543905
        assert np.sign(yaw_trajectory._trajectory_data.angular_displacement) == +1
        assert yaw_trajectory._trajectory_data.time_initial == time_initial
        
        self.assertAlmostEqual(yaw_trajectory.setpoint_driver.yaw, 0.0, delta=1e-4)

        # the angular distance should be decreasing
        last_angular_distance = yaw_trajectory._trajectory_data.angular_displacement
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
        assert np.sign(yaw_trajectory._trajectory_data.angular_displacement) == -1

        self.assertAlmostEqual(yaw_trajectory._trajectory_data.angular_displacement, -0.5)

        # the angular distance should be decreasing
        last_angular_distance = abs(yaw_trajectory._trajectory_data.angular_displacement)
        while yaw_trajectory._internal_state == yaw_trajectory.State.MOVING:
            send_update()
            yaw_setpoint = yaw_trajectory.setpoint_driver.yaw
            angular_distance = _find_angular_distance(yaw_setpoint, yaw_trajectory._yaw_target)
            x = angular_distance < last_angular_distance or angular_distance < 1e-6
            if not x:
                
                print(f"last_angular_distance: {last_angular_distance}")
                print(f"angular_distance: {angular_distance}")
                print(f"yaw_setpoint: {yaw_setpoint}")
                print(f"yaw_target: {yaw_trajectory._yaw_target}")
                assert x
            last_angular_distance = angular_distance
        
        
    
    @unittest.skip("???")
    @patch('dr_onboard_autonomy.states.components.trajectory.time', spec=True)
    def test_initial_yaw_uses_last_yaw_setpoint(self, mock_time: MagicMock):
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
            mode=FCUCopterMode.OFFBOARD,
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
    def test_initial_yaw_uses_sensed_yaw(self, mock_time: MagicMock):
        """Test the yaw trajectory's starting yaw is the IMU yaw
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
        drone.send_lla_setpoint(lla=(0,0,0), yaw=yaw_setpoint)

        yaw_trajectory = YawTrajectory(dr_state)

        sensor_messages = mock_all_drone_data(
            mode=FCUCopterMode.OFFBOARD,
            position=briarlla_from_lat_lon_alt(
                41.60667852830868,
                -86.35530652323709,
                229.0,
                is_amsl=True
            ),
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
        assert yaw_trajectory._trajectory_data.yaw_initial == 0.0
        assert yaw_trajectory._trajectory_data.yaw_final == yaw_trajectory._yaw_target
        assert yaw_trajectory._trajectory_data.angular_displacement == yaw_trajectory._yaw_target


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