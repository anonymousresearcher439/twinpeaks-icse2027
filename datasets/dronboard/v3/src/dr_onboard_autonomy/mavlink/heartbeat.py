import enum
from dataclasses import dataclass
from queue import Queue
from threading import Event, RLock, Thread
from typing import Callable, Mapping, Optional

import rospy
from time import sleep

from dr_onboard_autonomy.models.connection import ExternalConnection
from pymavlink.dialects.v20 import common as mavlink2
from .mavlink import MavlinkNode, MavState, MavType
from .mavlink_sender import MavlinkSender


class _MessageType(enum.Enum):
    SEND_NOW = enum.auto()
    STOP = enum.auto()
    GET_MAV_STATE = enum.auto()
    SET_MAV_STATE = enum.auto()


@dataclass
class _Message:
    message_type: _MessageType
    mav_state: MavState = None
    return_channel: Queue = None


class HeartbeatSender:
    def __init__(self,
                mavlink_sender: MavlinkSender,
                 component_type: MavType,
                 send_frequency: float = 4.0):
        self.mavlink_sender = mavlink_sender
        self.component_type = component_type.value
        self._send_frequency = send_frequency
        self._message_queue: Queue = Queue()
        self._stop_event = Event()

        self._sender_thread = Thread(target=self._run)
        self._timer_thread = Thread(target=self._run_timer)
        self._mavstate = MavState.UNINIT

        self._on_message = {
            _MessageType.SEND_NOW: self._on_send_now_message,
            _MessageType.GET_MAV_STATE: self._on_get_mav_state,
            _MessageType.SET_MAV_STATE: self._on_set_mav_state,
        }

    def start(self):
        self._sender_thread.start()
        self._timer_thread.start()

    def stop(self, await_stop=True):
        self._stop_event.set()
        if not self._sender_thread.is_alive():
            return
        message = _Message(message_type=_MessageType.STOP)
        self._message_queue.put(message)
        if await_stop:
            self._sender_thread.join()
    
    @property
    def mav_state(self) -> MavState:
        GET_MAV_STATE = _MessageType.GET_MAV_STATE
        msg = _Message(GET_MAV_STATE, return_channel=Queue())
        self._message_queue.put(msg)
        return msg.return_channel.get(timeout=1.0)
    
    @mav_state.setter
    def mav_state(self, value: MavState):
        SET_MAV_STATE = _MessageType.SET_MAV_STATE
        msg = _Message(SET_MAV_STATE, mav_state=value)
        self._message_queue.put(msg)


    def _run(self):
        def stop_callback():
            self.stop(await_stop=False)

        rospy.on_shutdown(stop_callback)

        while not rospy.is_shutdown():
            message: _Message = self._message_queue.get()
            if message.message_type == _MessageType.STOP:
                return
            on_message_method = self._on_message[message.message_type]
            on_message_method(message)
    
    def _run_timer(self):
        message = _Message(message_type=_MessageType.SEND_NOW)
        rate = rospy.Rate(self._send_frequency)
        while not rospy.is_shutdown() and not self._stop_event.is_set():
            try:
                if self._sender_thread.is_alive():
                    self._message_queue.put(message)
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def _on_send_now_message(self, _: _Message):
        heartbeat = mavlink2.MAVLink_heartbeat_message(
            type=self.component_type,
            autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
            base_mode=0,
            custom_mode=0,
            system_status=self._mavstate.value,
            mavlink_version=0,
        )
        self.mavlink_sender.send(heartbeat)
    
    def _on_get_mav_state(self, msg: _Message):
        msg.return_channel.put(self._mavstate)

    def _on_set_mav_state(self, msg: _Message):
        self._mavstate = msg.mav_state




class HeartbeatSender2:
    """Sends a heartbeat as the specified component type at the specified frequency via the provided
    communication channel

    The mav state initializes as an uninitialized system, so be sure to update the mav_state
    property to the appropriate mav state after initialization
    """
    def __init__(
            self,
            communication_channel: ExternalConnection,
            controller: MavlinkNode,
            component_type: MavType=MavType.ONBOARD_CONTROLLER,
            send_frequency: float = 1.0
    ):
        self._communication = communication_channel
        self._controller = controller
        self._component_type = component_type

        self._mavlink_protocol_handler = mavlink2.MAVLink(
            "", 
            self._controller.system_id,
            self._controller.component_id
        )

        self._exit_event: Event = Event()

        self._mav_state_lock: RLock = RLock()
        self._mav_state: MavState = MavState.UNINIT.value

        self._send_heartbeat_at_frequency_thread: Thread = Thread(
            target=self._manage_thread_shutdown,
            kwargs={
                "function_to_loop": self._send_heartbeat_at_frequency,
                "kwargs":{"frequency": send_frequency}
            },
            daemon=True
        )


    @property
    def mav_state(self) -> MavState:
        with self._mav_state_lock:
            return self._mav_state


    @mav_state.setter
    def mav_state(self, value: MavState):
        with self._mav_state_lock:
            self._mav_state = value


    def start(self):
        rospy.loginfo("HeartbeatSender2 - starting heartbeat sender")
        self._communication.start()
        self._send_heartbeat_at_frequency_thread.start()


    def stop(self):
        """Shuts down the heartbeat sender and internal threads"""
        self._exit_event.set()
        self._communication.close()

        rospy.loginfo("HeartbeatSender2 - shut down heartbeat sender")


    def _manage_thread_shutdown(self, function_to_loop: Callable, kwargs: Optional[Mapping]=None):
        '''Loops indefinitely on function_to_loop until exit event triggers break
        '''
        while True:
            if kwargs is not None:
                function_to_loop(**kwargs)
            else:
                function_to_loop()

            if self._exit_event.is_set():
                break


    def _pack_heartbeat(self) -> bytes:
        return(
            mavlink2.MAVLink_heartbeat_message(
                type=MavType.ONBOARD_CONTROLLER.value,
                autopilot=mavlink2.MAV_AUTOPILOT_INVALID,
                base_mode=0,
                custom_mode=0,
                system_status=self.mav_state,
                mavlink_version=0
            ).pack(self._mavlink_protocol_handler)
        )


    def _send_heartbeat(self):
        self._communication.send(self._pack_heartbeat())


    def _send_heartbeat_at_frequency(self, frequency: int=1):
        """Expects to be wrapped in while True loop and defaults to sending a heartbeat at 1 hz
        """
        self._send_heartbeat()
        # frequency is the heartbeat send rate
        sleep(1 / frequency)
