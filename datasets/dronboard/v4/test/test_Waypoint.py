
import math
import sys
import unittest
import inspect
from unittest.mock import (
    NonCallableMock,
    patch
)
from queue import Queue
import numpy as np

from droneresponse_mathtools import Lla, Pvector
from ruckig import InputParameter, Result, Ruckig, Trajectory


from dr_onboard_autonomy.airlease.protocol import AirLeaseOutcome
from dr_onboard_autonomy.models.drone import CopterParameters
from dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition
from dr_onboard_autonomy.states import Waypoint
from dr_onboard_autonomy.briar_helpers import (
    amsl_to_ellipsoid,
    BriarLla,
    convert_tuple_to_LlaDict,
    convert_Lla_to_LlaDict,
    magnitude,
    angle_between_vectors,
    check_angle_with_threshold,
    distance_is_less_than,
)
from dr_onboard_autonomy.message_senders import (
    AbstractMessageSender,
    ReusableMessageSenders,
)
from dr_onboard_autonomy.models.config import CameraConfig
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components import AirLease
from dr_onboard_autonomy.states.components.trajectory import KinematicState, TrajectoryGenerator, WaypointTrajectory

from .mock_types import mock_message_sender_position_message, mock_copter_drone, mock_position, mock_position_message, mock_shutdown_message, mock_state_kwargs, mock_mission_builder

from tf.transformations import unit_vector


WaypointModule = inspect.getmodule(Waypoint)
IncrementalWaypoint = WaypointModule.IncrementalWaypoint
WaypointState = WaypointModule.WaypointState
HandoffStatus = WaypointModule.HandoffStatus

def orthogonal_vector(forward, up, theta: float):
    """
    This function will return a vector that is orthogonal to the input vector v.
    The angle theta will determine the direction of the orthogonal vector.
    When theta is 0 we return the up vector.
    When theta is 90 we return the right vector
    when theta is 180 we return the down vector
    when theta is 270 we return the left vector

    Theta can be any value in degrees.
    """
    theta = math.radians(theta)
    forward = unit_vector(forward)
    up = unit_vector(up)
    right = np.cross(forward, up)
    right = unit_vector(right)
    vertical = np.cross(right, forward)
    vertical = unit_vector(vertical)

    return vertical * np.cos(theta) + right * np.sin(theta) 


class TestWaypoint(unittest.TestCase):
    def test_init(self):
        mission_builder = mock_mission_builder()

        args = mission_builder.build_kwargs({
            "waypoint": {
                "latitude": 41.60654653889745,
                "longitude": -86.35575685387629,
                "altitude": 240
            },
            "speed": 7.0,
            "stare_pitch": 45.0,
            "incremental_segment_time": 5.0,
            "incremental_handoff_percent": 40
        })
        waypoint = Waypoint(**args)

        expected_waypoint = BriarLla.from_args(41.60654653889745, -86.35575685387629, 240, is_amsl=True)
        self.assertEqual(waypoint.waypoint.amsl.tup, expected_waypoint.amsl.tup)
        self.assertEqual(waypoint.speed, 7.0)
        self.assertAlmostEqual(waypoint.stare_pitch, math.radians(45.0))
        self.assertEqual(waypoint.incremental_segment_time, 5.0)
        self.assertEqual(waypoint.incremental_handoff_percent, 40/100)
        self.assertIsNone(waypoint.stare_position)

    def test_name_gets_updated_in_status_message(self):
        name = "TESTING_WAYPOINT_NAME"
        mission_builder = mock_mission_builder()

        args = mission_builder.build_kwargs({
            "name": name,
            "waypoint": {
                "latitude": 41.60654653889745,
                "longitude": -86.35575685387629,
                "altitude": 240
            },
            "speed": 7.0,
            "stare_pitch": 45.0,
            "incremental_segment_time": 5.0,
            "incremental_handoff_percent": 40
        })
        assert args['name'] == name, "Name should be set in args"
        assert args['drone'] is not None
        assert args['drone'].data is not None

        waypoint = Waypoint(**args)
        self.assertEqual(waypoint.name, name)

        expected_waypoint = BriarLla.from_args(41.60654653889745, -86.35575685387629, 240, is_amsl=True)
        self.assertEqual(waypoint.waypoint.amsl.tup, expected_waypoint.amsl.tup)
        self.assertEqual(waypoint.speed, 7.0)
        self.assertAlmostEqual(waypoint.stare_pitch, math.radians(45.0))
        self.assertEqual(waypoint.incremental_segment_time, 5.0)
        self.assertEqual(waypoint.incremental_handoff_percent, 40/100)
        self.assertIsNone(waypoint.stare_position)

        waypoint.message_queue = Queue()
        waypoint.message_queue.put(mock_shutdown_message())
        waypoint.execute(None)
        # make sure that drone.data.name is correctly set
        self.assertEqual(waypoint.drone.data.state_name, name)
    
    def test_basics2(self):
        """run through the basic operation of the Waypoint state
        
        This test will will simulate flying a 50m path. The non-stop trajectory will take about 12.4167 seconds to complete.
        
        The goal is to require 4 segments. so the incremental segment time will be 6 seconds.
        
        1. handoff is None, endpoint is 6 seconds
        2. handoff is 2.4, segment ends at 8.4 seconds
        3. handoff is 4.8, segment ends at 10.8 seconds
        4. handoff is 7.2, segment ends at 12.4 seconds
        
        The handoff will be at 40% of the way along the path. The drone will fly at 5m/s.
        """
        mission_builder = mock_mission_builder()

        final_destination = BriarLla.from_dict({
            "latitude": 41.60654653889745,
            "longitude": -86.35575685387629,
            "altitude": 240
        }, is_amsl=True)
        start_position = final_destination.ellipsoid.lla.move_ned(-50, 0, 0)
        start_position = BriarLla.from_lla(start_position, is_amsl=False)

        incremental_segment_time = 6.12
        incremental_handoff_percent = 40
        speed = 5.0

        # lets create a trajectory to find:
        # 1. the expected handoff position
        # 2. the expected end position
        # 3. the expected duration

        inp: InputParameter = InputParameter(3)
        target_ned = start_position.ellipsoid.lla.distance_ned(final_destination.ellipsoid.lla)
        inp.target_position = [float(c) for c in target_ned]
        inp.target_velocity = [0.0, 0.0, 0.0]
        inp.target_acceleration = [0.0, 0.0, 0.0]

        inp.current_position = [0.0, 0.0, 0.0]
        inp.current_velocity = [0.0, 0.0, 0.0]
        inp.current_acceleration = [0.0, 0.0, 0.0]

        inp.max_velocity = [speed, speed, speed]
        constraints: CopterParameters = mission_builder.build_kwargs({})['drone'].params
        max_horizontal_acceleration = constraints.horizontal_acceleration_limit
        max_down_acceleration = constraints.downward_acceleration_limit
        max_up_acceleration = constraints.upward_acceleration_limit
        inp.max_acceleration = [
            max_horizontal_acceleration,
            max_horizontal_acceleration,
            max_down_acceleration
        ]
        inp.min_acceleration = [
            -1.0 * max_horizontal_acceleration,
            -1.0 * max_horizontal_acceleration,
            -1.0 * max_up_acceleration
        ]
        max_jerk = constraints.jerk_limit
        inp.max_jerk = [
            max_jerk,
            max_jerk,
            max_jerk
        ]

        otg = Ruckig(3)
        trajectory_calculator = Trajectory(3)

        # Calculate the trajectory in an offline manner
        calc_result = otg.calculate(inp, trajectory_calculator)
        if calc_result == Result.ErrorInvalidInput:
            raise Exception('Invalid input!')

        # now we can use the trajectory calculator to find the expected positions
        trajectory_calculator
        duration = trajectory_calculator.duration
        self.assertAlmostEqual(duration, 12.416666666380936)
        #handoff times
        # 2.4
        # 4.8
        # 7.2
        
        # end pos times
        # 6
        # 8.4 
        # 10.8
        # 12.4


    #TODO DO NOT SKIP THIS UNIT TEST
    @unittest.skip("TODO REWORK THIS UNIT TEST")
    def test_some_basics(self):
        """run through the basic operation of the Waypoint state
        
        This test will will simulate flying a 50m path. The non-stop trajectory will take about 12.4167 seconds to complete.
        
        The goal is to request 4 segments. so the incremental segment time will be 6 seconds.

        when we first start out the handoff position is None. we will calculate 
        an incremental endpoint that's 6 seconds away.

        We will then calculate the handoff position. The handoff position will be at 40% of the way along the path so the handoff position will be at 2.4 seconds away.

        Once we've established the endpoint and handoff position, we need to test the following:
        - receive setpoint messages
        - receive position messages
        
        1. handoff is None, endpoint is 6 seconds
        2. handoff is 2.4, segment ends at 8.4 seconds
        3. handoff is 4.8, segment ends at 10.8 seconds
        4. handoff is 7.2, segment ends at 12.4 seconds
        
        The handoff will be at 40% of the way along the path. The drone will fly at 5m/s.
        """
        mission_builder = mock_mission_builder()

        # define the key parameters once:
        speed = 5.0
        incremental_segment_time = 6.0
        incremental_handoff_percent = 40.0

        inc_handoff = incremental_handoff_percent / 100.0
        args = mission_builder.build_kwargs({
            "waypoint": {
                "latitude": 41.60654653889745,
                "longitude": -86.35575685387629,
                "altitude": 240
            },
            "speed": speed,
            "stare_pitch": 45.0,
            "incremental_segment_time": incremental_segment_time,
            "incremental_handoff_percent": incremental_handoff_percent
        })
        waypoint = Waypoint(**args)
        self.assertIsInstance(waypoint.waypoint, BriarLla)
        self.assertAlmostEqual(waypoint.stare_pitch, math.radians(45.0))
        self.assertIsInstance(waypoint.trajectory, WaypointTrajectory)
        self.assertIsInstance(waypoint.airlease, AirLease)
        self.assertIsInstance(waypoint.waypoint_state_machine, IncrementalWaypoint)

        # make sure waypoint_state_machine has it's fields initialized
        machine = waypoint.waypoint_state_machine
        self.assertEqual(machine.dr_state, waypoint)
        self.assertEqual(machine.drone, waypoint.drone)
        self.assertEqual(machine.trajectory, waypoint.trajectory)
        self.assertEqual(machine.airlease, waypoint.airlease)
        self.assertEqual(machine.speed, speed)
        self.assertEqual(machine.incremental_segment_time, incremental_segment_time)
        self.assertEqual(machine.incremental_handoff_percent, incremental_handoff_percent / 100.0)
        self.assertEqual(machine.state, WaypointState.READY_CHECK)
        
        # calculate the drone's starting position for this test
        waypoint_wgs84 = waypoint.waypoint.ellipsoid.lla
        drone_start = waypoint_wgs84.move_ned(-50, 0, 0) 
        
        # we need to mock some stuff
        waypoint.message_queue = Queue()

        # mock the trajectory component
        waypoint.trajectory = NonCallableMock(wraps=waypoint.trajectory)
        machine.trajectory = waypoint.trajectory
        waypoint.trajectory.state.return_value = WaypointTrajectory.State.HOLDING

        # mock the airlease component
        waypoint.airlease = NonCallableMock(wraps=waypoint.airlease)
        machine.airlease = waypoint.airlease
        waypoint.airlease.request_airspace.side_effect = list(range(10, 20))

        # track the handoff positions
        handoff_positions = []

        pos_msg = mock_position_message(drone_start.latitude, drone_start.longitude, drone_start.altitude)
        waypoint.message_queue.put(pos_msg)
        waypoint.message_queue.put(mock_shutdown_message())
        
        waypoint.execute(None)
        assert waypoint.trajectory.state() == WaypointTrajectory.State.HOLDING
        self.assertEqual(machine.state, WaypointState.REQUESTING)

        # check that we requested the airspace correctly
        waypoint.airlease.request_airspace.assert_called_once()
        args = waypoint.airlease.request_airspace.call_args.args
        waypoint.airlease.request_airspace.reset_mock()
        
        actual_airspace = args[0]
        actual_callback = args[1]

        self.assertEqual(len(actual_airspace), 1)
        self.assertEqual(actual_callback, machine.on_air_lease_outcome)
        
        # now let's dig into the airspace
        segment = actual_airspace[0]
        
        # make sure the start point is correct
        start = segment.start
        start = Lla(start.latitude, start.longitude, start.altitude)
        self.assertLess(start.distance(drone_start), 0.001) # make sure we start near the drone
        
        # make sure the end point is correct
        actual_airspace_end = segment.end
        actual_airspace_end = Lla(actual_airspace_end.latitude, actual_airspace_end.longitude, actual_airspace_end.altitude)

        # NOTE: we need to calcculate the expected endpoints...
        # create the trajectory calculator that will fly the entire trajectory
        # as if it were one continuous/smooth motion. For this test we are
        # making sure the "happy" path works as expected. So our motion with
        # incremental trajectories should match the motion of this continuous
        # trajectory
        trajectory_calculator, origin_wgs84, drone_pos = machine.trajectory_calculator()
        def find_endpoint(t):
            displacement, _, _ = trajectory_calculator.at_time(t)
            endpoint = origin_wgs84.move_ned(*displacement)
            return endpoint
        
        # here are the endpoints
        endpoints = [
            find_endpoint(6.0),
            find_endpoint(8.4),
            find_endpoint(10.8),
            find_endpoint(12.4),
        ]
        
        #NOTE: let's calculate the handoff position while we are at it
        handoff_points = [
            find_endpoint(2.4),
            find_endpoint(4.8),
            find_endpoint(7.2),
        ]

        # now back to making sure the endpoints are correct
        self.assertLess(actual_airspace_end.distance(endpoints[0]), 0.00001) # make sure we end near the waypoint
        
        airspace = machine.find_segment()
        self.assertEqual(airspace, actual_airspace)
        
        self.assertEqual(machine.state, WaypointState.REQUESTING)
        # now let's approve the airspace
        # we do this by updating the mock airlease component 
        machine.airlease.airspace = actual_airspace
        def mock_fly_to_waypoint(pos, _):
            if machine.trajectory.state() != WaypointTrajectory.State.MOVING:
                machine.trajectory.state.return_value = WaypointTrajectory.State.STARTING
            machine.trajectory.target_position = pos
        
        machine.trajectory.fly_to_waypoint.side_effect = mock_fly_to_waypoint
        machine.trajectory.reset_mock() 
        machine.on_air_lease_outcome(10, airspace, AirLeaseOutcome(approved=True, deadlock=False))
        # make sure we called the trajectory component
        machine.trajectory.fly_to_waypoint.assert_called_once()
        # check that we entered the FLYING state
        self.assertEqual(machine.state, WaypointState.FLYING)
        # check that we did the handoff calculation correctly
        self.assertEqual(machine.handoff_status, HandoffStatus.CALCULATING)

        # now let's mock the trajectory component's kinematic state method
        trajectory_time = 0.0
        def mock_kinematic_state(offset: float):
            nonlocal trajectory_time
            t = trajectory_time + offset
            displacement, velocity, acceleration = trajectory_calculator.at_time(t)
            drone_pos = pos_msg['data']
            return KinematicState(
                origin=drone_pos,
                position=[float(c) for c in displacement],
                velocity=[float(c) for c in velocity],
                acceleration=[float(c) for c in acceleration],
            )
        machine.trajectory.kinematic_state.side_effect = mock_kinematic_state
        machine.trajectory.state.return_value = WaypointTrajectory.State.MOVING

        self.assertEqual(machine.trajectory.state(), WaypointTrajectory.State.MOVING)
        
        waypoint.execute2(None)
        # make sure we called the trajectory component
        machine.trajectory.kinematic_state.assert_called_once()
        # make sure it was called with almost 2.4
        actual_offset = machine.trajectory.kinematic_state.call_args.args[0]
        self.assertAlmostEqual(actual_offset, 2.4)
        machine.trajectory.kinematic_state.reset_mock()
        # make sure we're in the FLYING state
        self.assertEqual(machine.state, WaypointState.FLYING)
        self.assertEqual(machine.handoff_status, HandoffStatus.READY)
        # make sure we found the expected handoff position
        handoff_wgs84 = machine.handoff_wgs84
        self.assertAlmostEqual(handoff_wgs84.latitude, handoff_points[0].latitude)
        self.assertAlmostEqual(handoff_wgs84.longitude, handoff_points[0].longitude)
        self.assertAlmostEqual(handoff_wgs84.altitude, handoff_points[0].altitude)
        # now let's call execute2 a few times to make sure nothing unexpected happens
        for _ in range(3):
            waypoint.execute2(None)
        self.assertEqual(machine.state, WaypointState.FLYING)
        self.assertEqual(machine.handoff_status, HandoffStatus.READY)

        # ok let's move forward in time
        trajectory_time = 2.4
        airspace = machine.find_segment()
        machine.trajectory.reset_mock()
        e = find_endpoint(8.4)
        # make sure e and airspace[0].end are the same
        e = Lla(e.latitude, e.longitude, e.altitude)
        actual_airspace_end = airspace[0].end
        actual_airspace_end = Lla(actual_airspace_end.latitude, actual_airspace_end.longitude, actual_airspace_end.altitude)
        a_n, a_e, a_d = drone_start.distance_ned(actual_airspace_end)
        a_n, a_e, a_d = round(a_n,2), round(a_e,2), round(a_d,2)
        print(f"actual endpoint coordinates:   {a_n}, {a_e}, {a_d}")
        e_n, e_e, e_d = drone_start.distance_ned(e)
        e_n, e_e, e_d = round(e_n,2), round(e_e,2), round(e_d,2)
        print(f"expected endpoint coordinates: {e_n}, {e_e}, {e_d}")
        self.assertLess(actual_airspace_end.distance(e), 0.0001)
        
        # let's update the position to be at the handoff position
        def process_message(msg):
            """This is what BaseState.execute() does with each message
            """
            waypoint.update_drone_data(msg)
            waypoint.message_data[msg["type"]] = msg["data"]
            waypoint.handlers.notify(msg)
            waypoint.execute2(None)

        drone_pos = find_endpoint(2.400001) # go just beyond the handoff position
        pos_msg = mock_position_message(drone_pos.latitude, drone_pos.longitude, drone_pos.altitude)
        process_message(pos_msg)
        
        # make sure machine.drone_wgs84 is the same as the position message
        drone_pos = machine.drone_wgs84
        self.assertAlmostEqual(drone_pos.latitude, pos_msg['data'].latitude)
        self.assertAlmostEqual(drone_pos.longitude, pos_msg['data'].longitude)
        self.assertAlmostEqual(drone_pos.altitude, pos_msg['data'].altitude)

        # make sure we're in the REQUESTING state
        self.assertEqual(machine.state, WaypointState.REQUESTING)
        machine.airlease.request_airspace.assert_called_once()
        # check that we requested the correct airspace
        actual_airspace, actual_callback = machine.airlease.request_airspace.call_args.args
        machine.airlease.request_airspace.reset_mock()

        self.assertEqual(len(actual_airspace), 1)
        self.assertEqual(actual_callback, machine.on_air_lease_outcome)
        # check that the start point is the drone's position
        start = actual_airspace[0].start
        start = Lla(start.latitude, start.longitude, start.altitude)
        self.assertLess(start.distance(drone_pos), 0.00001)
        # make sure the end point is correct
        actual_airspace_end = actual_airspace[0].end
        actual_airspace_end = Lla(actual_airspace_end.latitude, actual_airspace_end.longitude, actual_airspace_end.altitude)
        machine.trajectory.kinematic_state.assert_called_once()

        expected_endpoint = endpoints[1]
        a_n, a_e, a_d = drone_start.distance_ned(actual_airspace_end)
        a_n, a_e, a_d = round(a_n,2), round(a_e,2), round(a_d,2)
        print(f"actual endpoint coordinates:   {a_n}, {a_e}, {a_d}")
        e_n, e_e, e_d = drone_start.distance_ned(expected_endpoint)
        e_n, e_e, e_d = round(e_n,2), round(e_e,2), round(e_d,2)
        print(f"expected endpoint coordinates: {e_n}, {e_e}, {e_d}")
        self.assertLess(actual_airspace_end.distance(expected_endpoint), 0.00001)
        
        displacement, _, _ = waypoint.trajectory_calculator.at_time(incremental_segment_time * inc_handoff)
        ne, ee, de = displacement # north expected, east expected, down expected
        expected_handoff = drone_start.move_ned(ne, ee, de)
        na, ea, da = start.distance_ned(handoff_wgs84) # norath actual, east actual, down actual
        # before we check each component, make sure the distance is within 1
        # meter. This will cause the test to fail faster and display a more
        # informative error message
        self.assertLess(handoff_wgs84.distance(expected_handoff), 1.0) 
        self.assertAlmostEqual(ne, na)
        self.assertAlmostEqual(ee, ea)
        self.assertAlmostEqual(de, da)

        # let's do a Sanity check on the handoff location. It should be less than half the distance to the waypoint
        # so the distance from start to handoff should be less than half the distance from start to end
        start2handoff = start.distance(handoff_wgs84)
        start2end = start.distance(actual_airspace_end)
        self.assertLess(start2handoff, start2end / 2)

        # next let's see what happens when we call the callback given to the
        # airlease component the air lease component is suppose to call this
        # when we get a response from the air leasing service
        callback = args.args[1]

        # first make sure we don't start flying the trajectory when the air
        # lease is denied
        airspace_outcome = AirLeaseOutcome(
            approved=False,
            deadlock=False,
        )
        callback(10, actual_airspace, airspace_outcome)
        waypoint.trajectory.fly_to_waypoint.assert_not_called()

        # now let's approve the airspace and verify that we start flying the
        # trajectory
        airspace_outcome = AirLeaseOutcome(
            approved=True,
            deadlock=False,
        )
        callback(10, actual_airspace, airspace_outcome)
        waypoint.trajectory.fly_to_waypoint.assert_called()

        # make sure the destination and speed are correct
        args = waypoint.trajectory.fly_to_waypoint.call_args
        args, _ = args
        # check the destination
        destination: LlaPosition = args[0]
        destination = destination.to_wgs84_ellipsoid()
        destination = Lla(destination.latitude, destination.longitude, destination.altitude)
        self.assertLess(actual_airspace_end.distance(destination), 0.00001)
        # check the speed
        speed = args[1]
        self.assertEqual(speed, 5.0)

        # before moving on reset the mocks
        waypoint.trajectory.reset_mock()
        waypoint.airlease.reset_mock()

        # next we will simulate moving the drone along the path Let's provide a
        # position message that puts us **before** the handoff position
        pos_msg = mock_position_message(drone_start.latitude, drone_start.longitude, drone_start.altitude)

        # HACK to simulate the Waypoint state getting a position message
        # We do this because Waypoint.execute() is not running...
        waypoint.update_drone_data(pos_msg)
        outcome = waypoint.handlers.notify(pos_msg)
        self.assertIsNone(outcome)
        # make sure we did _NOT_ change the handoff position
        self.assertEqual(waypoint.handoff_wgs84, handoff_wgs84)
        # make sure we didn't call the mocks
        waypoint.trajectory.fly_to_waypoint.assert_not_called()
        waypoint.airlease.request_airspace.assert_not_called()

        # ok let's move past the handoff position
        drone_pos = handoff_wgs84.move_ned(1, -0.2, -0.001)
        setpoint_pos = handoff_wgs84.move_ned(1.5, 0.0, 0.0)
        setpoint_pos_ned = drone_start.distance_ned(setpoint_pos)
        drone_vel = (5.0, 0.0, 0.0)
        drone_accel = (0.0, 0.0, 0.0)
        waypoint.trajectory._context.trajectory_calculator.at_time.return_value = (setpoint_pos_ned, drone_vel, drone_accel)
        waypoint.trajectory._context.position_initial.ellipsoid.lla = drone_start
        pos_msg = mock_position_message(drone_pos.latitude, drone_pos.longitude, drone_pos.altitude)
        # HACK again
        waypoint.update_drone_data(pos_msg)
        outcome = waypoint.handlers.notify(pos_msg)
        self.assertIsNone(outcome)
        # make sure we updated the handoff position
        self.assertNotEqual(waypoint.handoff_wgs84, handoff_wgs84)
        # make sure asked for a new airspace
        waypoint.airlease.request_airspace.assert_called()
        
        self.assertIn(10, waypoint.segment_labels)
        self.assertIn(11, waypoint.segment_labels)

        # let's make sure we requested the right airspace

        args = waypoint.airlease.request_airspace.call_args
        actual_airspace = args.args[0]
        callback = args.args[1]
        # the actual airspace starts at the drone's position
        segment = actual_airspace[0]
        start = segment.start
        start = Lla(start.latitude, start.longitude, start.altitude)
        for start_component, drone_component in zip(start.lla, drone_pos.lla):
            self.assertAlmostEqual(start_component, drone_component)
        
        self.assertLess(start.distance(drone_pos), 0.00001) # make sure we start near the drone
        self.assertGreater(start.distance(setpoint_pos), 0.5) # make sure `segment.start`` is different from the setpoint

        actual_airspace_end = segment.end
        actual_airspace_end = Lla(actual_airspace_end.latitude, actual_airspace_end.longitude, actual_airspace_end.altitude)
        displacement, _, _ = waypoint.trajectory_calculator.at_time(incremental_segment_time)
        # remember the origin location for all trajectory calculations is the
        # drone's start position because we are replacing trajectories midway
        expected_end = drone_start.move_ned(*displacement)
        for expected_comp, end_comp in zip(expected_end.lla, actual_airspace_end.lla):
            self.assertAlmostEqual(expected_comp, end_comp)
        
        # next check the handoff position
        handoff_positions.append(waypoint.handoff_wgs84)
        handoff_wgs84 = waypoint.handoff_wgs84 # actual handoff position
        displacement, _, _ = waypoint.trajectory_calculator.at_time(incremental_segment_time * inc_handoff)
        expected_handoff = drone_start.move_ned(*displacement)
        # make sure the handoff correct
        # we do a basic distance check before checking the components to make
        # error messages more informative
        self.assertLess(handoff_wgs84.distance(expected_handoff), 0.01) 
        for expected_comp, handoff_comp in zip(expected_handoff.lla, handoff_wgs84.lla):
            self.assertAlmostEqual(expected_comp, handoff_comp)
        
        # sanity check, make sure the endpoint is north of the handoff position
        n_handoff2end, e_handoff2end, d_handoff2end = handoff_wgs84.distance_ned(actual_airspace_end) 
        self.assertGreater(n_handoff2end, 1.0)
        # make sure the others are close to zero (like within 0.001)
        self.assertLess(abs(e_handoff2end), 0.001) # within a millimeter
        self.assertLess(abs(d_handoff2end), 0.001)
        
        # now let's deny the airspace and make sure we don't start flying the new trajectory
        airspace_outcome = AirLeaseOutcome(
            approved=False,
            deadlock=False,
        )
        callback(11, actual_airspace, airspace_outcome)
        waypoint.trajectory.fly_to_waypoint.assert_not_called()

        # now let's approve the airspace and see if we start flying the trajectory
        airspace_outcome = AirLeaseOutcome(
            approved=True,
            deadlock=False,
        )
        callback(11, actual_airspace, airspace_outcome)
        waypoint.trajectory.fly_to_waypoint.assert_called()
        # make sure we provide the destination and speed
        args = waypoint.trajectory.fly_to_waypoint.call_args
        args, _ = args
        destination: LlaPosition = args[0]
        destination = destination.to_wgs84_ellipsoid()
        destination = Lla(destination.latitude, destination.longitude, destination.altitude)
        self.assertLess(actual_airspace_end.distance(destination), 0.00001)
        for expected_comp, destination_comp in zip(actual_airspace_end.lla, destination.lla):
            self.assertAlmostEqual(expected_comp, destination_comp)
        speed = args[1]
        self.assertEqual(speed, 5.0)

        # now let's move the drone so it's near the handoff position but not past it
        # first reset the mocks
        waypoint.trajectory.reset_mock()
        waypoint.airlease.reset_mock()

        # let's go south of the handoff position by 1 meter
        drone_pos: Lla = handoff_wgs84.move_ned(-1, 0.0, 0.0)
        pos_msg = mock_position_message(drone_pos.latitude, drone_pos.longitude, drone_pos.altitude)
        waypoint.update_drone_data(pos_msg)
        outcome = waypoint.handlers.notify(pos_msg)
        # make sure we didn't call the mocks
        waypoint.trajectory.fly_to_waypoint.assert_not_called()
        waypoint.airlease.request_airspace.assert_not_called()

        # now let's move the drone so it's past the handoff position
        drone_pos: Lla = handoff_wgs84.move_ned(0.1, 0.0, 0.0)
        pos_msg = mock_position_message(drone_pos.latitude, drone_pos.longitude, drone_pos.altitude)
        waypoint.update_drone_data(pos_msg)
        outcome = waypoint.handlers.notify(pos_msg)
        # make sure we didn't call the trajectory mock
        waypoint.trajectory.fly_to_waypoint.assert_not_called()
        # make sure we DID call the airlease mock
        waypoint.airlease.request_airspace.assert_called()

        # the handoff just updated
        handoff_positions.append(waypoint.handoff_wgs84)

        # now we provide the airspace
        args = waypoint.airlease.request_airspace.call_args
        actual_airspace = args.args[0]
        callback = args.args[1]
        segment = actual_airspace[0]
        
        airspace_outcome = AirLeaseOutcome(
            approved=True,
            deadlock=False,
        )
        callback(12, actual_airspace, airspace_outcome)
        self.assertIn(10, waypoint.segment_labels)
        self.assertIn(11, waypoint.segment_labels)
        self.assertIn(12, waypoint.segment_labels)
        waypoint.trajectory.fly_to_waypoint.assert_called()
        args = waypoint.trajectory.fly_to_waypoint.call_args
        args, _ = args
        trajectory_target: LlaPosition = args[0]
        trajectory_target = trajectory_target.to_wgs84_ellipsoid()
        trajectory_target = Lla(trajectory_target.latitude, trajectory_target.longitude, trajectory_target.altitude)
        trajectory_speed = args[1]
        self.assertLess(actual_airspace_end.distance(trajectory_target), 0.00001)
        self.assertEqual(trajectory_speed, 5.0)

        waypoint.trajectory.reset_mock()
        waypoint.airlease.reset_mock()
        

        # lets see the segment end
        actual_airspace_end = segment.end
        actual_airspace_end = Lla(actual_airspace_end.latitude, actual_airspace_end.longitude, actual_airspace_end.altitude)

        final_destination = LlaPosition(
            latitude=41.60654653889745,
            longitude=-86.35575685387629,
            altitude=240,
            datum_ref=DATUM_REFERENCE.AMSL
        )
        final_destination = final_destination.to_wgs84_ellipsoid()
        final_destination = Lla(final_destination.latitude, final_destination.longitude, final_destination.altitude)
        

        # lets find our handoff position
        previous_handoff = handoff_positions[-1]
        self.assertLess(previous_handoff.distance(waypoint.handoff_wgs84), 0.00001)
        
        self.assertEqual(waypoint.handoff_wgs84, "")
        handoff_positions.append(waypoint.handoff_wgs84)
        # make sure the new handoff position is north of the end position
        n,e,d = previous_handoff.distance_ned(handoff_wgs84)
        
        




    def test_is_past_handoff_01(self):
        """
        Test the function that determines when the drone has progressed far enough along the planned path that it should request permission for the next segment.
        
        To test this properly, we need to:

        1. Move the drone just past the limit position along the planned path
        2. Then add a small random deviation orthogonal to the path to simulate real-world conditions (Note: this is not implemented in this test... put another way this vector is zero in this test)
        3. Check if the function correctly determines that it's time to request the next segment
        """
        dest = BriarLla.from_args(
            latitude=41.60654653889745,
            longitude=-86.35575685387629,
            altitude=240, is_amsl=True
        )

        start = dest.ellipsoid.lla.move_ned(-100, 0, 0) # 100m south of destination
        start = BriarLla.from_lla(start, is_amsl=False)


        start2dest = dest.ellipsoid.lla.to_pvector().xyz - start.ellipsoid.lla.to_pvector().xyz
        start2dest_magnitude = np.linalg.norm(start2dest)
        start2dest_unit = start2dest /  start2dest_magnitude
        self.assertAlmostEqual(start2dest_magnitude, 100)

        mission_builder = mock_mission_builder()
        args = mission_builder.build_kwargs({
            "waypoint": dest.amsl.dict,
            "speed": 7.0,
            "stare_pitch": 45.0,
            "incremental_segment_time": 5.0,
            "incremental_handoff_percent": 40
        })
        waypoint = Waypoint(**args)


        drone_pos = start.ellipsoid.lla.to_pvector().xyz + start2dest_unit * 20.0001
        drone_wgs84 = Pvector(*drone_pos).to_lla()

        start_wgs84 = start.ellipsoid.lla

        handoff_ecef = start.ellipsoid.lla.to_pvector().xyz + start2dest_unit * 20
        handoff_wgs84 = Pvector(*handoff_ecef).to_lla()


        result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
        self.assertTrue(result)

    def test_is_past_handoff_02(self):
        """
        testing the function that determines when a drone has progressed far enough along the planned path that it should request permission for the next segment.
        
        To test this properly, we need to:

        1. Move the drone to the limit position along the planned path
        2. Check if the function correctly determines that it's time to request the next segment
        """
        dest = BriarLla.from_args(
            latitude=41.60654653889745,
            longitude=-86.35575685387629,
            altitude=240, is_amsl=True
        )

        start = dest.ellipsoid.lla.move_ned(-100, 0, 0) # 100m south of destination
        start = BriarLla.from_lla(start, is_amsl=False)

        start2dest = dest.ellipsoid.lla.to_pvector().xyz - start.ellipsoid.lla.to_pvector().xyz
        start2dest_magnitude = np.linalg.norm(start2dest)
        start2dest_unit = start2dest /  start2dest_magnitude
        self.assertAlmostEqual(start2dest_magnitude, 100)

        drone_pos = start.ellipsoid.lla.to_pvector().xyz + start2dest_unit * 20.000
        drone_wgs84 = Pvector(*drone_pos).to_lla()

        start_wgs84 = start.ellipsoid.lla

        handoff_ecef = start.ellipsoid.lla.to_pvector().xyz + start2dest_unit * 20
        handoff_wgs84 = Pvector(*handoff_ecef).to_lla()


        result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
        self.assertTrue(result)
    
    def test_is_past_handoff_03(self):
        """
        testing the function that determines when a drone has progressed far enough along the planned path that it should request permission for the next segment.
        
        In this test we will make sure the function returns False when the drone has not yet reached the handoff position.

        We will:

        1. Put the drone near the limit position along the planned path (but not past it)
        2. Check if the function correctly determines that it's time to request the next segment

        We expect the function to return False because the drone has not yet reached the handoff position.
        """
        dest = BriarLla.from_args(
            latitude=41.60654653889745,
            longitude=-86.35575685387629,
            altitude=240, is_amsl=True
        )

        start = dest.ellipsoid.lla.move_ned(-100, 0, 0) # 100m south of destination
        start = BriarLla.from_lla(start, is_amsl=False)

        start2dest = dest.ellipsoid.lla.to_pvector().xyz - start.ellipsoid.lla.to_pvector().xyz
        start2dest_magnitude = np.linalg.norm(start2dest)
        start2dest_unit = start2dest /  start2dest_magnitude
        self.assertAlmostEqual(start2dest_magnitude, 100)

        drone_pos = start.ellipsoid.lla.to_pvector().xyz + start2dest_unit * 19.999
        drone_wgs84 = Pvector(*drone_pos).to_lla()

        start_wgs84 = start.ellipsoid.lla

        handoff_ecef = start.ellipsoid.lla.to_pvector().xyz + start2dest_unit * 20
        handoff_wgs84 = Pvector(*handoff_ecef).to_lla()


        result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
        self.assertFalse(result)

    def test_is_past_handoff_04(self):
        """
        Test the function that determines when the drone has progressed far enough along the planned path that it should request permission for the next segment.
        
        In this test we will put the drone  just beyond the limit position but it will _NOT_ be perfectly aligned with the path. This will test the function's ability to handle real-world conditions where the drone may not be perfectly aligned with the path.

        To test this properly, we need to:

        1. Move the drone to just beyond the limit position along the planned path (the limit position is 20m along the path; we will put the drone beyond that by 1 micron)
        2. Then add a deviation that is  orthogonal to the planned path to simulate real-world conditions.
        3. Check if the function correctly determines that it's time to request the next segment

        We expect the function to return True because the drone has reached the handoff position.
        """
        dest = BriarLla.from_args(
            latitude=41.60654653889745,
            longitude=-86.35575685387629,
            altitude=240, is_amsl=True
        )

        start = dest.ellipsoid.lla.move_ned(-100, 0, 0) # 100m south of destination
        start = BriarLla.from_lla(start, is_amsl=False)


        start2dest = dest.ellipsoid.lla.to_pvector().xyz - start.ellipsoid.lla.to_pvector().xyz
        start2dest_magnitude = np.linalg.norm(start2dest)
        start2dest_unit = start2dest /  start2dest_magnitude
        self.assertAlmostEqual(start2dest_magnitude, 100)

        # this is our 1-D position along the path
        # we will move the drone to this position then add a small deviation orthogonal to the path
        # move the drone along the path:  20m  + 1 micron (to simulate going just beyond the handoff position)
        drone_path_pos = start.ellipsoid.lla.to_pvector().xyz + (start2dest_unit * (20.0 + 1e-6))
        drone_path_pos = Pvector(*drone_path_pos)
        up = drone_path_pos.to_nvector().xyz
        
        start_wgs84 = start.ellipsoid.lla
        handoff_ecef = start.ellipsoid.lla.to_pvector().xyz + start2dest_unit * 20
        handoff_wgs84 = Pvector(*handoff_ecef).to_lla()

        for angle in range(0, 360, 5):
            for linear_deviation in range(1, 100, 5):
                
                err_vector = orthogonal_vector(start2dest_unit, up, angle)
                self.assertAlmostEqual(np.linalg.norm(err_vector), 1.0)
                small_deviation = linear_deviation/100.0

                with self.subTest(angle=angle, linear_deviation=linear_deviation):
                    drone_pos = drone_path_pos.xyz + (err_vector * linear_deviation)
                    drone_wgs84 = Pvector(*drone_pos).to_lla()

                    result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
                    self.assertTrue(result, f"Failed at angle {angle} and linear deviation {linear_deviation}")

                with self.subTest(small_deviation=small_deviation, angle=angle):
                    drone_pos = drone_path_pos.xyz + (err_vector * small_deviation)
                    drone_wgs84 = Pvector(*drone_pos).to_lla()

                    result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
                    self.assertTrue(result, f"Failed at angle {angle} and linear deviation {small_deviation}")

    def test_is_past_handoff_05(self):
        """
        Test the function that determines when the drone has progressed far enough along the planned path that it should request permission for the next segment.
        
        In this test we will put the drone just BEFORE the limit position but it will _NOT_ be perfectly aligned with the path. This will test the function's ability to handle real-world conditions where the drone may not be perfectly aligned with the path.

        To test this properly, we need to:

        1. Move the drone to just before the limit position along the planned path (the limit position is 20m along the path; we will put the drone before that by 1 micron)
        2. Then add a deviation that is  orthogonal to the planned path to simulate real-world conditions.
        3. Check if the function correctly determines that it's NOT time to request the next segment

        We expect the function to return False because the drone has _NOT_ reached the handoff position yet.
        """
        dest = BriarLla.from_args(
            latitude=41.60654653889745,
            longitude=-86.35575685387629,
            altitude=240, is_amsl=True
        )

        start = dest.ellipsoid.lla.move_ned(-100, 0, 0) # 100m south of destination
        start = BriarLla.from_lla(start, is_amsl=False)


        start2dest = dest.ellipsoid.lla.to_pvector().xyz - start.ellipsoid.lla.to_pvector().xyz
        start2dest_magnitude = np.linalg.norm(start2dest)
        start2dest_unit = start2dest /  start2dest_magnitude
        self.assertAlmostEqual(start2dest_magnitude, 100)

        # this is our 1-D position along the path
        # we will move the drone to just before this position
        # then we add a small deviation orthogonal to the path
        # move the drone along the path:  20m - 1 micron (to simulate going just up to the handoff position but not past it)
        drone_path_pos = start.ellipsoid.lla.to_pvector().xyz + (start2dest_unit * (20.0 - 1e-6))
        drone_path_pos = Pvector(*drone_path_pos)
        up = drone_path_pos.to_nvector().xyz
        
        start_wgs84 = start.ellipsoid.lla
        handoff_ecef = start.ellipsoid.lla.to_pvector().xyz + (start2dest_unit * 20.0)
        handoff_wgs84 = Pvector(*handoff_ecef).to_lla()

        for angle in range(0, 361, 5):
            for linear_deviation in range(1, 100, 5):
                
                err_vector = orthogonal_vector(start2dest_unit, up, angle)
                self.assertAlmostEqual(np.linalg.norm(err_vector), 1.0)
                small_deviation = linear_deviation/100.0

                with self.subTest(angle=angle, linear_deviation=linear_deviation):
                    drone_pos = drone_path_pos.xyz + (err_vector * linear_deviation)
                    drone_wgs84 = Pvector(*drone_pos).to_lla()

                    result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
                    self.assertFalse(result, f"Failed at angle {angle} and linear deviation {linear_deviation}")

                with self.subTest(small_deviation=small_deviation, angle=angle):
                    drone_pos = drone_path_pos.xyz + (err_vector * small_deviation)
                    drone_wgs84 = Pvector(*drone_pos).to_lla()

                    result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
                    self.assertFalse(result, f"Failed at angle {angle} and linear deviation {small_deviation}")

    def test_is_past_handoff_06(self):
        """
        Test the function that determines when the drone has progressed far enough along the planned path that it should request permission for the next segment.
        
        In this test we will put the drone well BEFORE the limit position but it will _NOT_ be perfectly aligned with the path. This will test the function's ability to handle real-world conditions where the drone may not be perfectly aligned with the path.

        To test this properly, we need to:

        1. Move the drone to someplace before the limit position along the planned path (the limit position is 20m along the path; we will put the drone maybe 10 meters before that)
        2. Then add a deviation that is  orthogonal to the planned path to simulate real-world conditions.
        3. Check if the function correctly determines that it's NOT time to request the next segment

        We expect the function to return False because the drone has _NOT_ reached the handoff position yet.
        """
        dest = BriarLla.from_args(
            latitude=41.60654653889745,
            longitude=-86.35575685387629,
            altitude=240, is_amsl=True
        )

        start = dest.ellipsoid.lla.move_ned(-100, 0, 0) # 100m south of destination
        start = BriarLla.from_lla(start, is_amsl=False)


        start2dest = dest.ellipsoid.lla.to_pvector().xyz - start.ellipsoid.lla.to_pvector().xyz
        start2dest_magnitude = np.linalg.norm(start2dest)
        start2dest_unit = start2dest /  start2dest_magnitude
        self.assertAlmostEqual(start2dest_magnitude, 100)

        # this is our 1-D position along the path
        # we will move the drone to just before this position
        # then we add a small deviation orthogonal to the path
        # move the drone along the path:  10m + 1 micron (to simulate being far from the handoff position but not past it)
        drone_path_pos = start.ellipsoid.lla.to_pvector().xyz + (start2dest_unit * (10 + 1e-6))
        drone_path_pos = Pvector(*drone_path_pos)
        up = drone_path_pos.to_nvector().xyz
        
        start_wgs84 = start.ellipsoid.lla
        handoff_ecef = start.ellipsoid.lla.to_pvector().xyz + (start2dest_unit * 20.0)
        handoff_wgs84 = Pvector(*handoff_ecef).to_lla()

        for angle in range(0, 361, 5):
            for linear_deviation in range(1, 100, 5):
                
                err_vector = orthogonal_vector(start2dest_unit, up, angle)
                self.assertAlmostEqual(np.linalg.norm(err_vector), 1.0)
                small_deviation = linear_deviation/100.0

                with self.subTest(angle=angle, linear_deviation=linear_deviation):
                    drone_pos = drone_path_pos.xyz + (err_vector * linear_deviation)
                    drone_wgs84 = Pvector(*drone_pos).to_lla()

                    result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
                    self.assertFalse(result, f"Failed at angle {angle} and linear deviation {linear_deviation}")

                with self.subTest(small_deviation=small_deviation, angle=angle):
                    drone_pos = drone_path_pos.xyz + (err_vector * small_deviation)
                    drone_wgs84 = Pvector(*drone_pos).to_lla()

                    result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
                    self.assertFalse(result, f"Failed at angle {angle} and linear deviation {small_deviation}")
 
    def test_is_past_handoff_07(self):
        """
        Test the function that determines when the drone has progressed far enough along the planned path that it should request permission for the next segment.
        
        In this test we will put the drone  far beyond the limit position but it will _NOT_ be perfectly aligned with the path. This will test the function's ability to handle real-world conditions where the drone may not be perfectly aligned with the path.

        To test this properly, we need to:

        1. Move the drone to far beyond the limit position along the planned path (the limit position is 20m along the path; we will put the drone beyond that by 10 meters)
        2. Add a deviation that is  orthogonal to the planned path to simulate real-world conditions.
        3. Check if the function correctly determines that it's time to request the next segment

        We expect the function to return True because the drone progressed beyond the handoff position.
        """
        dest = BriarLla.from_args(
            latitude=41.60654653889745,
            longitude=-86.35575685387629,
            altitude=240, is_amsl=True
        )

        start = dest.ellipsoid.lla.move_ned(-100, 0, 0) # 100m south of destination
        start = BriarLla.from_lla(start, is_amsl=False)
        start_wgs84 = start.ellipsoid.lla

        start2dest = dest.ellipsoid.lla.to_pvector().xyz - start.ellipsoid.lla.to_pvector().xyz
        start2dest_magnitude = np.linalg.norm(start2dest)
        start2dest_unit = start2dest /  start2dest_magnitude
        self.assertAlmostEqual(start2dest_magnitude, 100)

        # this is our 1-D position along the path
        # we will move the drone to this position then add a small deviation orthogonal to the path
        # move the drone along the path:  30m  + 1 micron (to simulate going well beyond the handoff position)
        drone_path_pos = start.ellipsoid.lla.to_pvector().xyz + (start2dest_unit * (30.0 + 1e-6))
        drone_path_pos = Pvector(*drone_path_pos)
        up = drone_path_pos.to_nvector().xyz

        handoff_ecef = start.ellipsoid.lla.to_pvector().xyz + (start2dest_unit * 20)
        handoff_wgs84 = Pvector(*handoff_ecef).to_lla()

        for angle in range(0, 361, 5):
            for linear_deviation in range(1, 100, 5):
                
                err_vector = orthogonal_vector(start2dest_unit, up, angle)
                self.assertAlmostEqual(np.linalg.norm(err_vector), 1.0)
                small_deviation = linear_deviation/100.0

                with self.subTest(angle=angle, linear_deviation=linear_deviation):
                    drone_pos = drone_path_pos.xyz + (err_vector * linear_deviation)
                    drone_wgs84 = Pvector(*drone_pos).to_lla()

                    result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
                    self.assertTrue(result, f"Failed at angle {angle} and linear deviation {linear_deviation}")

                with self.subTest(small_deviation=small_deviation, angle=angle):
                    drone_pos = drone_path_pos.xyz + (err_vector * small_deviation)
                    drone_wgs84 = Pvector(*drone_pos).to_lla()

                    result = WaypointModule._is_past_handoff(start_wgs84, drone_wgs84, handoff_wgs84, dest.ellipsoid.lla)
                    self.assertTrue(result, f"Failed at angle {angle} and linear deviation {small_deviation}")
