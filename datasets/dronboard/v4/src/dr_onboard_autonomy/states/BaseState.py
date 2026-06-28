"""The BaseState module. All states extend the BaseState class in this module
"""
from typing import Optional, Type, Dict, Iterable

import numpy as np
import rospy
import smach

from dr_onboard_autonomy.airlease import AirLeaserModel
from dr_onboard_autonomy.message_senders import (RepeatTimer,
                                                 ReusableMessageSenders)
from dr_onboard_autonomy.models import CopterDrone
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components import (
    BatteryMonitor,
    HeartbeatStatusHandler,
    RcFailsafe,
    StatusMessage,
    TargetPosition,
    TaskCanceledTrigger,
    VisionApprove,
    VisionTrigger,
)
from dr_onboard_autonomy.models import (
    EnuYaw,
    RelativeAltitude,
    FCUState,
    IMU,
    NedVelocity,
    TimeStampedLlaPosition,
)

from .collections import MessageHandler, MessageQueue, MessageQueueImpl, PerformanceAnalysisQueue


class BaseState(smach.State):
    """All DR states extend this class. It has the execute method.
    """
    def __init__(
        self,
        air_lease_protocol_fsm: Optional[AirLeaserModel]=None,
        drone: Optional[CopterDrone]=None,
        reusable_message_senders: Optional[ReusableMessageSenders]=None,
        mqtt_client: Optional[MQTTClient]=None,
        local_mqtt_client: Optional[MQTTClient]=None,
        mqtt_cloud: Optional[MQTTClient]=None,
        heartbeat_handler: bool=True,
        trajectory_class: Optional['Type[TrajectoryGenerator]']=None,
        name: Optional[str]=None,
        data:Dict=None,
        **kwargs
    ):
        """
        All flying states should specify a trajectory_class so that the heartbeat component doesn't crash in case we lose connection
        """
        outcomes = kwargs["outcomes"]
        # make sure every state can have the error outcome so that shutdown works
        if "error" not in outcomes:
            outcomes.append("error")
        super().__init__(outcomes=outcomes)
        self.args = kwargs
        if 'uav_id' in kwargs:
            self.uav_id = kwargs['uav_id']
        if name is None:
            rospy.logwarn("BaseState: name is None, using class name as default")
            name = type(self).__name__
        self.air_lease_protocol_fsm = air_lease_protocol_fsm
        self.name = name
        self.drone = drone
        self.reusable_message_senders = reusable_message_senders
        self.local_mqtt_client = local_mqtt_client
        self.mqtt_client = mqtt_client
        self.mqtt_cloud = mqtt_cloud
        self.message_data = {}
        self._is_performance_analysis_mode = kwargs.get("performance_analysis", False)
        # self.message_queue = Queue()
        if self._is_performance_analysis_mode:
            perf_db = kwargs.get("performance_analysis_database", None)
            self.message_queue: MessageQueue = PerformanceAnalysisQueue(self, perf_db)
        else:
            self.message_queue: MessageQueue = MessageQueueImpl()
        self.message_senders = set()
        self.handlers = MessageHandler()
        self.data = data

        self.status_message = StatusMessage(self)

        shutdown_sender = self.reusable_message_senders.find("shutdown")
        self.message_senders.add(shutdown_sender)
        self.handlers.add_handler("shutdown", self.on_shutdown)

        self.trajectory: 'Optional[TrajectoryGenerator]' = None

        '''
        using human control outcome as flying state indicator
        '''
        self.rc_failsafe = None
        # TODO remove this line when we integrate the heartbeat microservice
        heartbeat_handler = False
        if "failsafe" in outcomes:
            self.rc_failsafe = RcFailsafe(self)
            if trajectory_class is not None:
                self.trajectory: 'Optional[TrajectoryGenerator]' = trajectory_class(self)
            if heartbeat_handler:
                self.hearbeat_status_handler = HeartbeatStatusHandler(self)
        
        # if we have all the data we need then initialize the vision trigger
        # The reason we're selectively initializing the vision trigger is because
        # in some test cases we don't want to provide all of these parameters
        # and we don't want to have to mock them out.
        # However, when we're running on a real drone or in the simulator we will
        # have all of these parameters.
        if self._is_home_available(kwargs) and "gimbal_calculator" in kwargs and "camera_config" in kwargs:
            self.vision_trigger = VisionTrigger(self, allow_transition="found" in outcomes, **kwargs)
        
        if "low_battery" in outcomes:
            self.battery_monitor = BatteryMonitor(self)
        
        if "vision_approve" in outcomes:
            self.vision_approve = VisionApprove(self, outcomes)

        if "target_position" in outcomes:
            self._target_position = TargetPosition(self)

        if "abort" in outcomes:
            # How to respond to a message sender
            self.message_senders.add(self.reusable_message_senders.find("abort"))
            self.handlers.add_handler("abort", self._on_abort)
        
        self.task_id = None
        if "task_canceled" in outcomes:
            task_id = kwargs.get("task_id", -1)
            self.task_canceled_trigger = TaskCanceledTrigger(self, task_id)
            self.task_id = kwargs.get("task_id", -1)


    def _on_abort(self, message):
        return "abort"

    def on_entry(self, userdata) -> Optional[str]:
        """The first thing that runs when we enter this state.

        This is the method calls the execute() method on other states, if the
        state implemented by running other states.
        """
        return None

    def on_exit(self, outcome: str, userdata) -> Optional[str]:
        """The last thing that runs as we exit this state.

        This method can call the execute() method on other states.

        Args:
            outcome: 
                the outcome that's causing this state to exit.
            user_data: 
                the user_data passed to the state's execute method.

        Returns: string specifying the new outcome or None
            if this method returns None then it's the same as returning the provided outcome string.
        """
        return None

    def execute(self, userdata):
        """The execute method receives messages and processes them"""
        self.drone.data.state_name = self.name
        self.drone.data.state_type = type(self).__name__

        outcome = self.on_entry(userdata)
        if outcome is not None:
            return outcome
        self.start_message_senders()

        while True:
            # msg_request_time = time.time()
            msg = self.message_queue.get()
            # msg_recv_time = time.time()

            # How long did we waiting to receive the next msg?
            # msg_wait_time = msg_recv_time - msg_request_time


            # update drone.data
            self.update_drone_data(msg)
            self.message_data[msg["type"]] = msg["data"]
            outcome = self.handlers.notify(msg)
            if outcome is not None:
                self.acknowledge_message(msg)
                return self._final(outcome, userdata)

            outcome = self.execute2(userdata)
            self.acknowledge_message(msg)
            if outcome is not None:
                return self._final(outcome, userdata)
            
            # How long did we spend processing the msg?
            # msg_processing_time = time.time() - msg_recv_time

            # log this in csv with a special prefix
            # line = f"@@@{time.time():.23f},{self.name},{type(self).__name__},{msg['type']},{msg_wait_time:.23f},{msg_processing_time:.23f},{len(self.message_queue)}"
            # rospy.logdebug(line)
    
    def update_drone_data(self, message):
        """Update the drone data with the message data
        """
        assert self.drone is not None, "Drone data is not set. Make sure to pass a drone instance to the BaseState constructor."
        msg_type = message["type"]
        rospy.logdebug(f"update_drone_data - message type: '{msg_type}' data: '{message['data']}'")
        if msg_type == "battery":
            self.drone.data.battery = message["data"]
        elif msg_type == "state":
            state: FCUState = message["data"]
            self.drone.data.armed = state.armed
            self.drone.data.connected = state.connected
            self.drone.data.mode = state.mode
            self.drone.data.status = state.status
            self.drone.data.landed_status = state.landed_status
        elif msg_type == "imu":
            imu: IMU = message["data"]
            self.drone.data.acceleration_linear = imu.acceleration_linear
            self.drone.data.attitude.add_attitude(imu.time, imu.attitude)
        elif msg_type == "compass_hdg":
            heading: EnuYaw = message["data"]
            self.drone.data.heading = heading
        elif msg_type == "velocity":
            velocity: NedVelocity = message["data"]
            self.drone.data.velocity = velocity
            north, east = velocity.north, velocity.east
            self.drone.data.ground_speed = np.linalg.norm([east, north])
        elif msg_type == "position":
            position: TimeStampedLlaPosition = message["data"]
            self.drone.data.location.add_position(position.time, position)
        elif msg_type == "relative_altitude":
            rel_alt: RelativeAltitude = message["data"]
            self.drone.data.relative_altitude = rel_alt
        elif msg_type == "gps_status":
            self.drone.data.gps_status = message["data"]
        elif msg_type == "target_attitude":
            self.drone.data.target_attitude = message["data"]
        elif msg_type == "vibration":
            self.drone.data.vibration = message["data"]
        elif msg_type == "rc_out":
            self.drone.data.rc_out = message["data"]
        elif msg_type == "ekf_status":
            self.drone.data.ekf_status = message["data"]
        else:
            rospy.logdebug(f"update_drone_data - unknown message type: {msg_type}")


            
    
    def _final(self, outcome: Optional[str], userdata) -> str:
        """This method is called when we exit a state.

        This method is called after the on_exit method, but before we return
        from the execute method. It's called regardless of the outcome of the
        on_exit method.

        Args:
            outcome: 
                the outcome that's causing this state to exit.
            exit_outcome: 
                the outcome returned by the on_exit method.
            user_data: 
                the user_data passed to the state's execute method.
        """
        self.stop_message_senders()
        if self._is_performance_analysis_mode:
            self.message_queue.finish_processing()
        exit_outcome = self.on_exit(outcome, userdata)
        if exit_outcome is not None:
            return exit_outcome
        return outcome

    def execute2(self, userdata):
        """Method that runs every time we receive a message that hasn't caused
        a state transition
        """
        return None
    
    def execute_substate(self, state: 'BaseState', userdata):
        outcome = state.execute(userdata)
        self.drone.data.state_name = self.name
        self.drone.data.state_type = type(self).__name__
        return outcome

    def start_message_senders(self):
        """Starts the message senders

        This tells the message senders to start passing their data to this
        state.
        """
        for sender in self.message_senders:
            sender.start(self.message_queue.put)

    def stop_message_senders(self):
        """Stops all the message senders.
        
        This gets called before exiting this state.
        """
        for sender in self.message_senders:
            sender.stop()

    def acknowledge_message(self, msg):
        """Calls the done_func if the message came from a reliable message
        sender.
        """
        if "done" in msg:
            done_func = msg["done"]
            done_func()


    def on_shutdown(self, message):
        return "error"    
    
    def _is_home_available(self, kwargs) -> bool:
        """Check if the home keyword arg is provided and valid
        """
        if "home" not in kwargs:
            return False
        home = kwargs["home"]
        if not home:
            return False
        if not isinstance(home, dict):
            return False
        if "latitude" not in home or "longitude" not in home or "altitude" not in home:
            return False
        # make sure we have a float for each value
        coordinates = [
            home["latitude"],
            home["longitude"],
            home["altitude"],
        ]
        return all(isinstance(coord, float) for coord in coordinates)
    
    def _process_outcomes(self, kwargs:Dict, state_outcomes:Iterable=set(), default_outcomes:Iterable=set(), unwanted_outcomes:Iterable=set()):
        """Process the outcomes for the state

        Args:
            kwargs (Dict) This is the kwargs that are passed to the state's init method. 
            state_outcomes (Iterable) The outcomes that are specific to this state
            default_outcomes (Iterable) The non-state specific outcomes. For example the STANDARD_TRANSITIONS_FLYING
            unwanted_outcomes (Iterable) The outcomes we want to remove. This is used to implement the states that we add by default. For example we need this to implement AbortHover or ReturnToRecharge 
                If no unwanted outcomes are provided we will check if the kwargs provide this.
        """
        outcomes = set(kwargs.get("outcomes", []))
        
        default_outcomes = set(default_outcomes)
        state_outcomes = set(state_outcomes)
        unwanted_outcomes = set(unwanted_outcomes)

        # TODO figure out if we should do this:
        # if not unwanted_outcomes:
        #     unwanted_outcomes = kwargs.get("unwanted_outcomes", unwanted_outcomes)


        required_outcomes = default_outcomes.union(state_outcomes)

        # TODOD should we remove unwanted outcomes from required_outcomes (a subset)? Or should we remove them from outcomes?
        # Do we want the mission to be able to add some unwanted outcomes?
        required_outcomes = required_outcomes - unwanted_outcomes
        
        outcomes.update(required_outcomes)

        return list(outcomes)

from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator


