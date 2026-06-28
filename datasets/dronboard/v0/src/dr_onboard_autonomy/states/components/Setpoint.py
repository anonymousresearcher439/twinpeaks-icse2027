from enum import Enum, auto
from typing import Tuple, Union
from dr_onboard_autonomy.mavros_layer import MAVROSDrone

from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states import BaseState, MessageHandler


RealNumber = Union[float, int]

class SetpointType(Enum):
    LLA = auto()
    NED_POSITION = auto()
    NED_VELOCITY = auto()


class Setpoint:
    class State(Enum):
        UNKNOWN = auto()
        STARTING = auto()
        RUNNING = auto()
        STOPPED = auto()
    

    def __init__(self, state: BaseState):
        self.dr_state = state
        self.drone: MAVROSDrone = state.drone
        self.message_senders = state.message_senders
        self.reusable_message_senders = state.reusable_message_senders
        self.handlers = state.handlers

        # setup message senders and callbacks
        flight_controller_state_sensor = self.reusable_message_senders.find("state")
        self.message_senders.add(flight_controller_state_sensor)
        self.handlers.add_handler("state", self.on_state_message)
        
        self.message_senders.add(RepeatTimer("setpoint", 0.1))
        self.handlers.add_handler("setpoint", self.on_setpoint_message)

        # data that describes the setpoint command we send to the drone
        self._setpoint_type = SetpointType.NED_VELOCITY
        self._setpoint_value = (0, 0, 0)
        self._setpoint_yaw = None

        # internal state machine
        self._state = Setpoint.State.UNKNOWN
        self._send_count = 0
    
    def start(self):
        self._state = Setpoint.State.UNKNOWN
        self._send_count = 0
        
        self.velocity = (0, 0, 0)
        self._setpoint_yaw = None
    
    def stop(self):
        self._state = Setpoint.State.STOPPED

    # alt is AMSL
    @property
    def lla(self):
        if self._setpoint_type != SetpointType.LLA:
            return None
        return self._setpoint_value

    # alt is AMSL
    @lla.setter
    def lla(self, p: Tuple[float, float, float]):
        self._setpoint_value = p
        self._setpoint_type = SetpointType.LLA
    
    @property
    def ned(self):
        if self._setpoint_type != SetpointType.NED_POSITION:
            return None
        return self._setpoint_value

    @ned.setter
    def ned(self, pos: Tuple[float, float, float]):
        self._setpoint_type = SetpointType.NED_POSITION
        self._setpoint_value = pos
    
    @property
    def velocity(self):
        if self._setpoint_type != SetpointType.NED_VELOCITY:
            return None
        return self._setpoint_value
    
    @velocity.setter
    def velocity(self, ned: Tuple[RealNumber, RealNumber, RealNumber]):
        self._setpoint_type = SetpointType.NED_VELOCITY
        self._setpoint_value = tuple([float(x) for x in ned])
    
    @property
    def yaw(self):
        return self._setpoint_yaw
    
    @yaw.setter
    def yaw(self, radians: float):
        self._setpoint_yaw = radians
    
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
        
        def send_lla():
            self.drone.send_setpoint(lla=target, yaw=yaw, is_yaw_set=is_yaw_set)
        
        def send_ned_position():
            self.drone.send_setpoint(ned_position=target, yaw=yaw, is_yaw_set=is_yaw_set)
        
        def send_ned_velocity():
            self.drone.send_setpoint(ned_velocity=target, yaw=yaw, is_yaw_set=is_yaw_set)

        send_function_table = {
            SetpointType.LLA: send_lla,
            SetpointType.NED_POSITION: send_ned_position,
            SetpointType.NED_VELOCITY: send_ned_velocity,
        }
        send_function = send_function_table[self._setpoint_type]
        send_function()

    def on_state_message(self, message):

        def unknown():
            if message['data'].mode == "OFFBOARD":
                self._state = Setpoint.State.RUNNING
            else:
                self._state = Setpoint.State.STARTING
                self._send_count = 0
        
        def starting():
            if self._send_count >= 10:
                self.drone.set_mode("OFFBOARD")
                self._state = Setpoint.State.RUNNING
        
        def running():
            if message['data'].mode != "OFFBOARD":
                self.stop()
        
        def stopped():
            pass

        function_table = {
            Setpoint.State.UNKNOWN: unknown,
            Setpoint.State.STARTING: starting,
            Setpoint.State.RUNNING: stopped,
            Setpoint.State.STOPPED: stopped,
        }
        method = function_table[self._state]
        method()
