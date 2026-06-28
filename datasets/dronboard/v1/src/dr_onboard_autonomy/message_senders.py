#!/usr/bin/env python3

from queue import Queue
import copy
import json
import socket
import time
from os import pipe
from select import select
from threading import Event, Lock, RLock, Thread
from enum import Enum, auto
from functools import partial
from typing import Union

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from mavros_msgs.msg import EstimatorStatus, ExtendedState, RCIn, State
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from std_msgs.msg import Float64, String
from geometry_msgs.msg import TwistStamped

from dr_onboard_autonomy.mqtt_client import MQTTClient


class AbstractMessageSender:
    def __init__(self, name):
        """All message senders need a name. The name is used to find the MessageHandler that need to run in response to a message."""
        self.name = name

    def start(self, to_active_state):
        """Starts forwarding new messages to the state machine.


        When the message sender wants to pass data to the state machine, it creates a dict to hold the data.
        The dict will always look like the following:

            example_message = {
                'type': 'message_sender_name',
                'data': 'example data could be anything'
            }

        The message sender then calls the given `to_state_machine` argument like so:

            to_state_machine(example_message)


        :param to_state_machine: The callback function that takes one argument, a dict. This function delivers the message to the state. This could be `state_machine.message_queue.put`
        :type to_state_machine: Callable
        """
        pass

    def stop(self):
        """Stops forwarding messages to the state machine."""
        pass


# We are using a message-driven finite state machine.
# Some messages come from ROS.
# This class takes ROS messages and passes them to the state machine
#
# It has a few important methods
# The start method creates the Subscriber
# The stop method unregisters the Subscriber
#
# The message sender has these states:
# - stopped
# - started
# - sending
#
# The ROSMessageMessageSender starts in the `stopped and not sending` state
#   the message sender is not connected to ROS in this state
# When you call start_subscriber() a ROS subscriber is connected to the topic and when
#   the message sender receives messages from a ROS topic it does not pass it along to a state.
# When you call start, you give it the callback that passes the data along to the current state.
#   from this point on, ROS messages will make their way to the active state
class ROSMessageSender(AbstractMessageSender):
    def __init__(self, name, topic, TopicType):
        super().__init__(name)
        self.topic = topic
        self.TopicType = TopicType
        self.sub = None
        self.lock = RLock()
        self.current_state = "stopped"

    def ros_callback(self, data):
        message = {"type": self.name, "data": data}
        self.send_message(message)

    def start_subscriber(self):
        with self.lock:
            self.sub = rospy.Subscriber(self.topic, self.TopicType, self.ros_callback)
            rospy.logdebug(f"ROS '{self.name}' message sender subscribed")

            self._change_state("started")
            rospy.logdebug(f"ROS '{self.name}' message sender started")

    def stop_subscriber(self):
        with self.lock:
            self.sub.unregister()
            rospy.logdebug(f"ROS '{self.name}' message sender UNREGISTERED")
            self.sub = None
            self._change_state("stopped")

    def start(self, to_state_machine):
        with self.lock:
            # TODO remame
            self.to_state_machine = to_state_machine
            if self.current_state == "stopped":
                self.start_subscriber()
            self._change_state("sending")

    def stop(self):
        self._change_state("stopped")

    def send_message(self, message):
        with self.lock:
            if self.current_state == "sending":
                self.to_state_machine(message)

    def _change_state(self, new_state):
        with self.lock:
            self.current_state = new_state


class CommandMessageSender(ROSMessageSender):
    def __init__(self):
        super().__init__("commands", "/birdy0/commands", String)
        self.is_armed = None
        self.takeoff = None
        self.altitudereached = None
        self.done = None
        self.complete = None

    def ros_callback(self, data):
        self.is_armed = data == "arm"
        self.takeoff = data == "takeoff"
        self.altitudereached = data == "hover"
        self.done = data == "land"
        self.complete = data == "finish"

        command_state = {
            "is_armed": self.is_armed,
            "takeoff": self.takeoff,
            "altitudereached": self.altitudereached,
            "done": self.done,
            "complete": self.complete,
        }

        message = {"type": self.name, "data": command_state}
        self.send_message(message)


class RepeatTimer(AbstractMessageSender):
    def __init__(self, name, time_limit):
        super().__init__(name)
        self.time_limit = time_limit

        self._is_running = False
        self.lock = Lock()

        self.thread = None
        self.to_state_machine = None
        self.sock_read = None
        self.sock_write = None

    def _run_timer(self, name, sock_read, time_limit):
        # To be sure that this method is thread safe
        # it only uses parameters, local variables, method calls and function calls

        last_time = time.monotonic()
        while self.is_running():
            read = [sock_read]
            write = []
            ex = [sock_read]

            delta_t = time.monotonic() - last_time
            time_remaining = time_limit - delta_t

            r, _, e = select(read, write, ex, max(0.0, time_remaining))

            assert sock_read not in e
            if sock_read in r:
                _ = sock_read.recv(2048)

            delta_t = time.monotonic() - last_time
            if delta_t > time_limit:
                message = {"type": name, "data": delta_t}
                self.put_message(message)
                last_time = time.monotonic()

    def put_message(self, message):
        with self.lock:
            if self._is_running:
                self.to_state_machine(message)

    def is_running(self):
        with self.lock:
            return self._is_running

    def start(self, to_state_machine):
        with self.lock:
            assert self.thread is None
            self._is_running = True
            self.sock_read, self.sock_write = socket.socketpair()
            self.to_state_machine = to_state_machine
            self.thread = Thread(
                target=self._run_timer,
                daemon=True,
                args=(self.name, self.sock_read, self.time_limit),
            )
            self.thread.start()

    def stop(self):
        with self.lock:
            if not self._is_running:
                return
            self._is_running = False

        # wake up the timer thread
        self.sock_write.send(b"\x00")
        # Wait for the timer thread to finish
        self.thread.join()

        # cleanup
        self.sock_read.close()
        self.sock_write.close()

        self.thread = None
        self.sock_read = None
        self.sock_write = None
        self.to_state_machine = None


class ShutdownMessageSender(AbstractMessageSender):
    def __init__(self):
        super().__init__("shutdown")
        self.lock = Lock()
        self.to_active_state = None
        self._is_running = True
        self.shutdown_message = {"type": "shutdown", "data": None}
        rospy.on_shutdown(self._rospy_stop)

    def start(self, to_active_state):
        with self.lock:
            self.to_active_state = to_active_state
            if not self._is_running:
                self.to_active_state(self.shutdown_message)

    def stop(self):
        with self.lock:
            self.to_active_state = None

    def _rospy_stop(self):
        with self.lock:
            self._is_running = False
            if self.to_active_state is not None:
                self.to_active_state(self.shutdown_message)


class MQTTMessageSender(AbstractMessageSender):
    def __init__(self, name, topic, mqtt_client: MQTTClient):
        super().__init__(name)
        self.topic = topic
        self.mqtt_client = mqtt_client
        self.lock = RLock()
        self.current_state = "stopped"

    def _mqtt_callback(self, client, userdata, mqtt_payload):
        # TODO should we parse the mqtt_payload as json?
        message = {"type": self.name, "data": mqtt_payload}
        self.send_message(message)

    def send_message(self, message):
        with self.lock:
            if self.current_state == "sending":
                self.to_state_machine(message)

    def start_subscriber(self):
        with self.lock:
            self.mqtt_client.subscribe(self.topic, self._mqtt_callback)
            rospy.loginfo(f"MQTT '{self.name}' message sender subscribed")
            self._change_state("started")
            rospy.loginfo(f"MQTT '{self.name}' message sender started")

    def start(self, to_active_state):
        with self.lock:
            self.to_state_machine = to_active_state
            if self.current_state == "stopped":
                self.start_subscriber()
            self._change_state("sending")

    def stop(self):
        self._change_state("stopped")

    _change_state = ROSMessageSender._change_state


class ReliableMessageSender(AbstractMessageSender):
    """Wraps an ordinary message sender to create a reliable one.

    To the smach states, this looks like an ordinary message sender.
    To the message sender being wrapped, this looks like a smach state.

    Besides start_loop and stop_loop, its safe to call all other public methods in any order. The exact behavior of each operation depends on the
    private state of this object.

    Every public method gets turned into a command object and queued. A worker thread
    receives each command, and does the operation specified.
    """

    class State(Enum):
        NOT_STARTED = auto()
        GATHERING_MESSAGES = auto()
        PASSING_MESSAGES = auto()
        DONE = auto()

    class Command(Enum):
        START = auto()
        DELIVER_MESSAGE = auto()
        ACKNOWLEDGE_MESSAGE = auto()
        STOP = auto()
        EXIT = auto()

    def __init__(self, message_sender):
        super().__init__(message_sender.name)
        self.message_sender = message_sender
        self.current_state = self.State.NOT_STARTED
        self.thread = Thread(target=self._run)
        # TODO consider setting the maxsize of the command_queue to a small value like 5 to detect
        # message overrun. Right now, the command_queue doesn't have a maxsize to avoid a possible
        # deadlock. if the queue were full and the worker thread stopped then a call to stop_loop
        # would dead lock at self.command_queue.put({self.Command.EXIT: True}).
        self.command_queue = Queue()
        self.messages = list()
        self.stop_lock = Lock()
        self.to_active_state = None
        self._next_message_id = 0

    def start_loop(self):
        """Start the worker thread and start the wrapped message sender to start giving us messages"""

        # This is the only public method not implimented with a command for the worker thread.
        self.thread.start()
        rospy.on_shutdown(self.stop_loop)

    def stop_loop(self, timeout=None):
        """Tell the worker thread to exit, and join the worker thread

        When the timeout argument is not None it should be a float that specifies how long to wait
        for blocking operations.
        """
        # This lock protects the case that too many threads call stop at the same time
        with self.stop_lock:
            if self.thread.is_alive():
                self.command_queue.put({self.Command.EXIT: True}, timeout=timeout)
        self.thread.join(timeout=timeout)

    def start(self, to_active_state):
        assert self.thread.is_alive()
        command = {
            self.Command.START: to_active_state,
        }
        self.command_queue.put(command)

    def stop(self):
        command = {self.Command.STOP: True}
        self.command_queue.put(command)

    def acknowledge_message(self, message_id):
        command = {self.Command.ACKNOWLEDGE_MESSAGE: message_id}
        self.command_queue.put(command)

    def deliver_message(self, message):
        command = {self.Command.DELIVER_MESSAGE: message}
        self.command_queue.put(command)

    def _run(self):
        self.current_state = self.State.GATHERING_MESSAGES

        # We start the message sender here and route all its messages to our deliver_message method.
        # We're putting these calls in this method to avoid a race condition. We can be more certain
        # that these methods are called once and in the correct order since the the worker thread
        # does it.
        self.message_sender.start(self.deliver_message)

        # create the states of our internal state machine
        # This code was partially auto-generated
        # for each state, we specify the method we want to run for each command.
        state_machine = {
            self.State.NOT_STARTED: {
                self.Command.START: None,  # This case should be impossible
                self.Command.DELIVER_MESSAGE: None,  # This case should be impossible
                self.Command.ACKNOWLEDGE_MESSAGE: None,  # This case should be impossible
                self.Command.STOP: None,  # This case should be impossible
                self.Command.EXIT: None,  # This case should be impossible
            },
            self.State.GATHERING_MESSAGES: {
                self.Command.START: self._gathering_messages_start_command,
                self.Command.DELIVER_MESSAGE: self._gathering_messages_deliver_message_command,
                self.Command.ACKNOWLEDGE_MESSAGE: self._gathering_messages_acknowledge_message_command,
                self.Command.STOP: self._gathering_messages_stop_command,  # this should not be called
                self.Command.EXIT: self._exit_command,
            },
            self.State.PASSING_MESSAGES: {
                self.Command.START: self._passing_messages_start_command,  # this should not be called
                self.Command.DELIVER_MESSAGE: self._passing_messages_deliver_message_command,
                self.Command.ACKNOWLEDGE_MESSAGE: self._passing_messages_acknowledge_message_command,
                self.Command.STOP: self._passing_messages_stop_command,
                self.Command.EXIT: self._exit_command,
            },
            self.State.DONE: {
                self.Command.START: None,  # This case should be impossible
                self.Command.DELIVER_MESSAGE: None,  # This case should be impossible
                self.Command.ACKNOWLEDGE_MESSAGE: None,  # This case should be impossible
                self.Command.STOP: None,  # This case should be impossible
                self.Command.EXIT: None,  # This case should be impossible
            },
        }

        while self.current_state is not self.State.DONE:
            state_functions = state_machine[self.current_state]
            cmd_message = self.command_queue.get()
            for cmd in cmd_message:
                cmd_func = state_functions[cmd]
                cmd_data = cmd_message[cmd]
                cmd_func(cmd_data)
        self.message_sender.stop()

    def _exit_command(self, _):
        self.current_state = self.State.DONE

    def _gathering_messages_start_command(self, to_active_state):
        self.to_active_state = to_active_state
        if self.messages:
            self._send_message(self.messages[0])
        self.current_state = self.State.PASSING_MESSAGES

    def _gathering_messages_deliver_message_command(self, message):
        self._buffer_new_message(message)

    def _gathering_messages_acknowledge_message_command(self, message_id):
        # This is an odd case. We are not passing messages to a smach state. But someone is telling
        # us that a message we've previously sent has now been processed.
        # We will find the message and remove it so that it doesn't get sent in the future
        self._remove_acknowledged_message(message_id)

    def _gathering_messages_stop_command(self, _):
        rospy.logwarn(
            "message_sender.stop() was called but this message sender is already stopped"
        )

    def _passing_messages_start_command(self, to_state_machine):
        # this is odd. We are already passing messages to a smach state. But we're being told to
        # start sending messages to a new smach state. Did a smach state transition without telling
        # its message senders to stop?
        rospy.logwarn(
            "message_sender.start() was called but this message sender is already started. Did a smach state transition without telling its message senders to stop? This message sender will start passing messages to the new destination"
        )
        self._gathering_messages_start_command(to_state_machine)

    def _passing_messages_deliver_message_command(self, message):
        # we are actively sending messages to a smach state. If there are no older messages awaiting
        # delivery, we should send this one now. Otherwise we should queue it for delivery later.
        is_only_messages = not self.messages
        self._buffer_new_message(message)

        if is_only_messages:
            self._send_message(message)

    def _passing_messages_acknowledge_message_command(self, message_id):
        # we are actively sending messages to a smach state and the smach state is telling us that
        # it has finished processing a message.
        # We need to ensure the acknowledged message dosen't get sent again.
        self._remove_acknowledged_message(message_id)

        # If there are more messages awaiting delivery, we should send the next one now.
        if self.messages:
            self._send_message(self.messages[0])

    def _passing_messages_stop_command(self, _):
        self.current_state = self.State.GATHERING_MESSAGES

    def _remove_acknowledged_message(self, message_id):
        message_index = self._index_of_message_id(message_id)
        if message_index >= 0:
            acknowledged_msg = self.messages.pop(message_index)
            # TODO consider if we want to logdebug this
            rospy.logdebug("message acknowledged", acknowledged_msg)

    def _index_of_message_id(self, message_id):
        # TODO do we need to check all the messages? Maybe we can get by only checking the first one
        for i, msg in enumerate(self.messages):
            if "message_id" in msg:
                if msg["message_id"] == message_id:
                    return i
        return -1

    def _buffer_new_message(self, message):
        message["message_id"] = self._get_next_message_id()
        message["done"] = partial(self.acknowledge_message, message["message_id"])
        self.messages.append(message)

    def _send_message(self, message):
        self.to_active_state(copy.copy(message))

    def _get_next_message_id(self):
        msg_id = self._next_message_id
        self._next_message_id += 1
        return msg_id


class MockMessageSender(AbstractMessageSender):
    def __init__(self, name):
        """All message senders need a name. The name is used to find the MessageHandler that need to run in response to a message."""
        self.name = name

    def start(self, to_active_state):
        rospy.logwarn(f"Cannot start message sender: '{self.name}'! Message sender does not exist")

    def stop(self):
        """Stops forwarding messages to the state machine."""
        rospy.logwarn(f"Cannot stop message sender: '{self.name}'! Message sender does not exist")


class ReusableMessageSenders:
    def __init__(self):
        self.all_message_senders = {}
        self.reliable_message_senders = []

    def populate(self):
        ros_message_senders = [
            ("position", "mavros/global_position/global", NavSatFix),
            ("relative_altitude", "mavros/global_position/rel_alt", Float64),
            ("battery", "mavros/battery", BatteryState),
            ("state", "mavros/state", State),
            ("extended_state", "mavros/extended_state", ExtendedState),
            ("diagnostics", "/diagnostics", DiagnosticArray),
            ("estimator_status", "mavros/estimator_status", EstimatorStatus),
            ("imu", "mavros/imu/data", Imu),
            ("compass_hdg", "mavros/global_position/compass_hdg", Float64),
            ("velocity", "mavros/global_position/raw/gps_vel", TwistStamped),
            ("rcin", "mavros/rc/in", RCIn),
        ]
        for name, topic, TopicType in ros_message_senders:
            self.all_message_senders[name] = ROSMessageSender(name, topic, TopicType)

        self.all_message_senders["commands"] = CommandMessageSender()
        self.all_message_senders["shutdown"] = ShutdownMessageSender()

    def find(self, name):
        if name in self.all_message_senders:
            return self.all_message_senders[name]
        else:
            rospy.logwarn(f"Could not find message sender named '{name}'")
            return MockMessageSender(name)

    def add(self, message_sender):
        name = message_sender.name
        self.all_message_senders[name] = message_sender
        if type(message_sender) == ReliableMessageSender:
            self.reliable_message_senders.append(message_sender)
            message_sender.start_loop()

    def stop(self):
        for message_sender in self.reliable_message_senders:
            message_sender.stop_loop()


class HeartbeatMessageSender(AbstractMessageSender):
    """
    MQTT message sender that interprets heartbeat messages and outputs a heartbeat status
    
    Expects MQTT heartbeat messages from ground control in the following format:
    topic = "heartbeat"
    data = {
        "heartbeat" : 15.234
        "service_id" : 12
    }

    Sends MQTT heartbeat status messages on local mqtt in the following format:
    topic = "heartbeat_status"
    data = {
        "status" : <status : hover, continue, rtl>
    }
    """
    repeat_timer_loop: int = 1

    def __init__(
        self,
        mqtt_client : MQTTClient,
        hover_threshold : int,
        rtl_threshold : int
    ):
        '''
        _on_heartbeat_message runs on mqtt thread separate from the RepeatTimer thread where 
        _on_heartbeat_check runs. Using a thread lock to keep functions running simultaneaously
        '''
        super().__init__("heartbeat_status")
        self._lock = RLock()
        self.heartbeat_service_id: Union[int, None] = None
        self._latest_ground_heartbeat: int = 0
        self._last_heartbeat_confirmed: int = 0
        self.last_time_confirmed: float = time.monotonic()
        self._mqtt_client = mqtt_client
        self.to_active_state = None

        self._mqtt_client.subscribe(
            topic="heartbeat",
            callback=self._on_heartbeat_message
        )

        '''
        add hover_threshold check so not < 2x RepeatTimer
        '''
        if hover_threshold < 2 * self.repeat_timer_loop:
            self.hover_threshold = 2 * self.repeat_timer_loop
            rospy.logwarn(
                "HeartbeatMessageSender - defaulted hover_threshold to minimum allowable: %s",
                2 * self.repeat_timer_loop
            )
        else:
            self.hover_threshold = hover_threshold

        '''
        make sure rtl_threshold >= hover_threshold
        '''
        if rtl_threshold < self.hover_threshold:
            self.rtl_threshold = self.hover_threshold
            rospy.logwarn(
                "HeartbeatMessageSender - defaulted rtl_threshold to match hover_threshold"
            )
        else:
            self.rtl_threshold = rtl_threshold

        self._heartbeat_check = RepeatTimer(
            name="heartbeat_check",
            time_limit=self.repeat_timer_loop
        )
    
    def start(self, to_active_state):
        """Starts forwarding new messages to the state machine.


        When the message sender wants to pass data to the state machine, it creates a dict to hold the data.
        The dict will always look like the following:

            example_message = {
                'type': 'message_sender_name',
                'data': 'example data could be anything'
            }

        The message sender then calls the given `to_state_machine` argument like so:

            to_state_machine(example_message)


        :param to_state_machine: The callback function that takes one argument, a dict. This function delivers the message to the state. This could be `state_machine.message_queue.put`
        :type to_state_machine: Callable
        """
        with self._lock:
            self.to_active_state = to_active_state


    def start_heartbeat(self):
        rospy.logdebug("HeartbeatMessageSender - starting heartbeat check")
        self._heartbeat_check.start(self._on_heartbeat_check)


    def stop_heartbeat(self):
        rospy.logdebug("HeartbeatMessageSender - stopping heartbeat check")
        self._heartbeat_check.stop()


    def _on_heartbeat_message(self, client, userdata, mqtt_payload):
        with self._lock:
            data = json.loads(mqtt_payload.payload)
            if data["service_id"] != self.heartbeat_service_id:
                '''
                re-initialize heartbeat logic on new heartbeat service id
                '''
                rospy.logdebug("HeartbeatMessageSender - received new heartbeat service id")
                self.heartbeat_service_id = data["service_id"]
                self._last_heartbeat_confirmed = data["heartbeat"]
                self._latest_ground_heartbeat = data["heartbeat"]
                self.last_time_confirmed = time.monotonic()
            else:
                rospy.logdebug(
                    f"HeartbeatMessageSender - received new heartbeat {data['heartbeat']}"
                )
                self._latest_ground_heartbeat = data["heartbeat"]

    def _output_message(self, status_value):
        if self.to_active_state == None:
            rospy.logdebug(f"HeartbeatMessageSender - skipping {status_value} message (no active state)")
            return

        rospy.logdebug(f"HeartbeatMessageSender - sending {status_value} message")
        message = {
            "type": "heartbeat_status",
            "data": {
                "status" : status_value
            }
        }
        self.to_active_state(message)

    def _on_heartbeat_check(self, message):
        with self._lock:
            if self._last_heartbeat_confirmed < self._latest_ground_heartbeat:
                self._last_heartbeat_confirmed = self._latest_ground_heartbeat
                self.last_time_confirmed = time.monotonic()

            if self._send_rtl_status():
                self._output_message("rtl")

            if self._send_hover_status():
                self._output_message("hover")

            if self._send_continue_status():
                self._output_message("continue")

    def _send_hover_status(self):
        return (
            (time.monotonic() - self.last_time_confirmed) > self.hover_threshold
            and
            (time.monotonic() - self.last_time_confirmed) <= self.rtl_threshold
        )
            

    def _send_rtl_status(self):
        return (time.monotonic() - self.last_time_confirmed) > self.rtl_threshold


    def _send_continue_status(self):
        return ((time.monotonic() - self.last_time_confirmed) < self.hover_threshold)



