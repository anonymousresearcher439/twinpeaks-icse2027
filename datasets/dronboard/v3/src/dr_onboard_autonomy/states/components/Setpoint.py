from enum import Enum, auto
from typing import Tuple, Union, Optional
from dr_onboard_autonomy.logutils import DebounceLogger

import rospy
from tf.transformations import euler_from_quaternion

from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.models import CopterDrone, FCUCopterMode, FCUState, IMU
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.models import NedVelocity, NedPosition, LlaPosition, DATUM_REFERENCE, EnuYaw


RealNumber = Union[float, int]

class SetpointType(Enum):
    LLA = auto()
    NED_POSITION = auto()
    NED_VELOCITY = auto()

    @staticmethod
    def is_position_setpoint(setpoint_type: 'SetpointType') -> bool:
        return setpoint_type in [SetpointType.LLA, SetpointType.NED_POSITION]


class Setpoint:
    class State(Enum):
        UNKNOWN = auto()
        STARTING = auto()
        PENDING_MODE_CHANGE = auto()
        RUNNING = auto()
        STOPPED = auto()
    
    TIMER_MESSAGE_NAME = "setpoint"

    TYPE_DATA  = 'setpoint_type'
    VALUE_DATA = 'setpoint_value'
    YAW_DATA   = 'setpoint_yaw'

    def __init__(self, state: BaseState):
        self.dr_state = state
        self.drone: CopterDrone = state.drone
        self.message_senders = state.message_senders
        self.reusable_message_senders = state.reusable_message_senders
        self.handlers = state.handlers

        # setup message senders and callbacks
        flight_controller_state_sensor = self.reusable_message_senders.find("state")
        self.message_senders.add(flight_controller_state_sensor)
        self.handlers.add_handler("state", self.on_state_message)

        self.message_senders.add(RepeatTimer(Setpoint.TIMER_MESSAGE_NAME, 1.0/25.0))

        # Before we add self.on_setpoint_message to self.handlers, we need to
        # understand the order of operations...
        #
        # We want to calculate the setpoint values right before creating
        # a setpoint message and sending it to the flight controller. To
        # achieve this, we first run some functions to calculate the
        # setpoint data, followed by a function that sends the setpoint
        # message. These functions are triggered each time we process a
        # “setpoint” message.
        #
        # There are four components reacting to each setpoint message:
        # (1) the LLA trajectory, (2) the gimbal component, (3) the yaw
        # trajectory, and (4) the setpoint sender.
        #
        # The LLA trajectory component runs first with priority 1. It
        # calculates the drone’s position (latitude, longitude, and
        # altitude).
        #
        # The gimbal component runs 2nd with priority 2. It calculates
        # the drone’s yaw target. The gimbal component depends on the
        # position data from the LLA trajectory component to accurately
        # calculate the drone’s target yaw. Note the target yaw is the
        # ideal yaw and not the same as the yaw value we put in the
        # setpoint message.
        #
        # the yaw trajectory component runs 3rd with priority 3. It
        # calculates the yaw setpoint. The yaw setpoint follows a
        # trajectory and changes slowly over time. The yaw trajectory
        # ends with the drone facing the target yaw. This calculation
        # depends on the gimbal component.
        #
        # Finally, the setpoint sender runs last with priority 10. It
        # builds the setpoint message and sends it to the flight
        # controller. The setpoint sender depends on the outputs from
        # both the LLA trajectory and yaw trajectory to build the final
        # setpoint message.
        self.handlers.add_handler(
            Setpoint.TIMER_MESSAGE_NAME,
            self.on_setpoint_message,
            can_transition=False,
            priority=10
        )

        self.message_senders.add(self.reusable_message_senders.find("imu"))
        self.handlers.add_handler("imu", self.on_imu_message)

        # data that describes the setpoint command we send to the drone
        self._setpoint_type = SetpointType.NED_VELOCITY
        self._setpoint_value = NedVelocity(0, 0, 0)
        self._setpoint_yaw = None
        self._default_yaw = None

        # internal state machine
        self._state = Setpoint.State.UNKNOWN
        self._send_count = 0

        self.debouce_logger = DebounceLogger()
        self._data = state.data

    def start(self, restore_previous_setpoint=False):
        self._state = Setpoint.State.UNKNOWN
        self._send_count = 0

        self.velocity = (0, 0, 0)
        self._setpoint_yaw = None

        if restore_previous_setpoint:
            self._restore_setpoint()
    
    def _save_setpoint(self):
        self._data[Setpoint.TYPE_DATA] = self._setpoint_type
        self._data[Setpoint.VALUE_DATA] = self._setpoint_value
        self._data[Setpoint.YAW_DATA] = self._setpoint_yaw
    
    def _restore_setpoint(self):
        # No previous setpoint to restore
        if Setpoint.VALUE_DATA not in self._data:
            rospy.loginfo("Setpoint: No previous setpoint to restore")
            return
    
        # in case we have a position setpoint, restore it
        # TODO: figure out if we should restore a velocity setpoint too
        if SetpointType.is_position_setpoint(self._data[Setpoint.TYPE_DATA]):
            rospy.loginfo(f"Setpoint: Restoring previous setpoint in {self.dr_state.name}")
            self._setpoint_type = self._data[Setpoint.TYPE_DATA]
            self._setpoint_value = self._data[Setpoint.VALUE_DATA]
        else:
            rospy.logwarn(f"Setpoint: Refusing to restore previous setpoint because it was a velocity setpoint in {self.dr_state.name}.")
            rospy.logwarn(f"Setpoint: setpoint_type = {self._setpoint_type}, setpoint_value = {self._setpoint_value}") # should be the defaults SetpointType.NED_VELOCITY and (0, 0, 0)
        
        self._setpoint_yaw = self._data[Setpoint.YAW_DATA]
        rospy.loginfo(f"Setpoint: Restored previous yaw {self._setpoint_yaw} in {self.dr_state.name}")

    
    def stop(self):
        self._state = Setpoint.State.STOPPED

    # alt is AMSL
    @property
    def lla(self) -> Optional[LlaPosition]:
        if self._setpoint_type != SetpointType.LLA:
            return None
        return self._setpoint_value

    # alt is AMSL
    @lla.setter
    def lla(self, p: Union[LlaPosition, Tuple[float, float, float]]):
        if isinstance(p, LlaPosition):
            lla_pos = p
        else:
            lla_pos = LlaPosition(p[0], p[1], p[2], datum_ref=DATUM_REFERENCE.AMSL)
        lla_pos = lla_pos.to_amsl()
        self._setpoint_value = lla_pos
        self._setpoint_type = SetpointType.LLA
    
    @property
    def ned(self) -> NedPosition:
        if self._setpoint_type != SetpointType.NED_POSITION:
            return None
        return self._setpoint_value

    @ned.setter
    def ned(self, pos: Union[NedPosition, Tuple[float, float, float]]):
        if isinstance(pos, NedPosition):
            ned_pos = pos
        else:
            ned_pos = NedPosition(pos[0], pos[1], pos[2])
        self._setpoint_type = SetpointType.NED_POSITION
        self._setpoint_value = ned_pos
    
    @property
    def velocity(self) -> NedVelocity:
        if self._setpoint_type != SetpointType.NED_VELOCITY:
            return None
        return self._setpoint_value
    
    @velocity.setter
    def velocity(self, ned: Union[NedVelocity, Tuple[RealNumber, RealNumber, RealNumber]]):
        if isinstance(ned, NedVelocity):
            ned_vel = ned
        else:
            ned_vel = NedVelocity(float(ned[0]), float(ned[1]), float(ned[2]))
        self._setpoint_type = SetpointType.NED_VELOCITY
        self._setpoint_value = ned_vel
    
    @property
    def yaw(self) -> Optional[EnuYaw]:
        """
        The yaw setpoint (units in radians) in ENU. If None, the drone should maintain
        its existing yaw that we read from the IMU.

        This specifies the aircraft's yaw direction in ENU. This is equivalent
        to the drones compass direction except the angle is around the up axis
        and the drone faces east when yaw is 0.


        For example, the drone will look North when yaw is pi/2, and South when yaw is -pi/2

        The direction of rotation should be the one with the shorted angular displacement.
        For example, if the drone is almost facing West (yaw = 99% * pi) and the
        yaw setpoint is -99% * pi then the drone should turn 0.06 radians (about 3.6 degrees)
        around the up axis.

        However, if the drone is facing south (yaw = -pi/2) and the yaw setpoint
        is North-West (yaw = 3/4 * pi) then the drone should rotate 135 degrees
        around the down axis.

        Note, you should never assign a large change in yaw.
        The drone will spin way too fast. Instead, you should gradually change
        the yaw setpoint over time. See `dr_onboard_autonomy.states.components.trajectory.YawTrajectory`
        """
        return self._setpoint_yaw

    @yaw.setter
    def yaw(self, radians: Optional[EnuYaw]):
        if radians is not None:
            radians = radians
        self._setpoint_yaw = radians

    def on_imu_message(self, message):
        imu: IMU = message['data']
        q = imu.attitude
        tup = q.x, q.y, q.z, q.w
        euler = euler_from_quaternion(tup)
        self._default_yaw = EnuYaw(euler[2])

    def on_setpoint_message(self, message):

        def unknown():
            self.send_setpoint()

        def starting():
            self.send_setpoint()
            self._send_count = self._send_count + 1

        def running():
            self.send_setpoint()
            self._send_count = self._send_count + 1

        def stopped():
            pass

        # given our state, what function do we run?
        state_function_table = {
            Setpoint.State.UNKNOWN: unknown,
            Setpoint.State.STARTING: starting,
            Setpoint.State.RUNNING: running,
            Setpoint.State.STOPPED: stopped,
        }
        func = state_function_table[self._state]
        func()

    def send_setpoint(self):
        target = self._setpoint_value
        is_yaw_set = self._setpoint_yaw is not None

        yaw = 0.0
        if is_yaw_set:
            yaw = self._setpoint_yaw
        else:
            yaw = self._default_yaw

        if yaw is None:
            # skip sending a setpoint if we don't have a yaw value
            # this should never happen.
            # the setpoint message arrives every 42ms, and the imu message
            # arrives every 20ms so this might happen if we change the setpoint
            # frequency

            # TODO remove this line and handle the case we don't have a default yaw in drone.send_setpoint
            rospy.logwarn("Setpoint: skipping setpoint because yaw is None")
            return

        def send_lla():
            # write log message that we are sending a setpoint
            msg = "Setpoint: sending setpoint: lla={}, yaw={}".format(target, round(yaw, 3))
            self.debouce_logger.info(msg, 1.0, 1)
            self.drone.send_lla_setpoint(lla=target, yaw=yaw)

        def send_ned_position():
            # write log message that we are sending a setpoint
            msg = "Setpoint: sending setpoint: ned={}, yaw={}".format(target, round(yaw, 3))
            self.debouce_logger.info(msg, 1.0, 2)
            self.drone.send_ned_setpoint(ned_position=target, yaw=yaw)

        def send_ned_velocity():
            # write log message that we are sending a velocity setpoint
            msg = "Setpoint: sending setpoint: velocity={}, yaw={}".format(target, round(yaw, 3))
            self.debouce_logger.info(msg, 1.0, 3)
            self.drone.send_ned_velocity_setpoint(
                ned_velocity=target,
                yaw=yaw
            )

        send_function_table = {
            SetpointType.LLA: send_lla,
            SetpointType.NED_POSITION: send_ned_position,
            SetpointType.NED_VELOCITY: send_ned_velocity,
        }
        send_function = send_function_table[self._setpoint_type]
        send_function()
        if self._data:
            self._save_setpoint()

    def on_state_message(self, message):
        drone_state: FCUState = message["data"]

        def unknown():
            if drone_state.mode == FCUCopterMode.OFFBOARD:
                self._state = Setpoint.State.RUNNING
            else:
                self._state = Setpoint.State.STARTING
                self._send_count = 0

        def starting():
            if self._send_count >= 35:
                self.drone.set_fcu_mode(FCUCopterMode.OFFBOARD)
                self._state = Setpoint.State.RUNNING
        
        def pending_mode_change():
            if drone_state.mode == FCUCopterMode.OFFBOARD:
                self._state = Setpoint.State.RUNNING

        def running():
            if drone_state.mode != FCUCopterMode.OFFBOARD:
                self.stop()

        def stopped():
            pass

        function_table = {
            Setpoint.State.UNKNOWN: unknown,
            Setpoint.State.STARTING: starting,
            Setpoint.State.PENDING_MODE_CHANGE: pending_mode_change,
            Setpoint.State.RUNNING: stopped,
            Setpoint.State.STOPPED: stopped,
        }
        method = function_table[self._state]
        method()
