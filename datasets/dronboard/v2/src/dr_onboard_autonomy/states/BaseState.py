"""The BaseState module. All states extend the BaseState class in this module
"""
from typing import Optional, Type, Dict
from dr_onboard_autonomy.states.components import TaskCanceledTrigger

import rospy
import smach

from dr_onboard_autonomy.air_lease import AirLeaseService
from dr_onboard_autonomy.message_senders import (RepeatTimer,
                                                 ReusableMessageSenders)
from dr_onboard_autonomy.models import CopterDrone
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components import (
    HeartbeatStatusHandler,
    RcFailsafe,
    StatusMessage2,
    TargetPosition,
    UpdateSpacialData,
    VisionTrigger,
)

from .collections import MessageHandler, MessageQueue, MessageQueueImpl, PerformanceAnalysisQueue


class BaseState(smach.State):
    """All DR states extend this class. It has the execute method.
    """
    def __init__(
        self,
        air_lease_service: Optional[AirLeaseService]=None,
        drone: Optional[CopterDrone]=None,
        reusable_message_senders: Optional[ReusableMessageSenders]=None,
        mqtt_client: Optional[MQTTClient]=None,
        local_mqtt_client: Optional[MQTTClient]=None,
        heartbeat_handler: bool=True,
        trajectory_class: 'Type[TrajectoryGenerator]' =None,
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
            name = type(self).__name__
        self.air_lease_service = air_lease_service
        self.name = name
        self.drone = drone
        self.reusable_message_senders = reusable_message_senders
        self.local_mqtt_client = local_mqtt_client
        self.mqtt_client = mqtt_client
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

        self.status_message = StatusMessage2(self)

        self.airlease_cleanup_timer = RepeatTimer("airlease_cleanup_timer", 1)
        self.message_senders.add(self.airlease_cleanup_timer)
        self.handlers.add_handler("airlease_cleanup_timer", self._airlease_cleanup)

        shutdown_sender = self.reusable_message_senders.find("shutdown")
        self.message_senders.add(shutdown_sender)
        self.handlers.add_handler("shutdown", self.on_shutdown)

        self.trajectory: 'Optional[TrajectoryGenerator]' = None

        '''
        using human control outcome as flying state indicator
        '''
        # TODO remove this line when we integrate the heartbeat microservice
        heartbeat_handler = False
        if "human_control" in outcomes:
            self._rc_failsafe_component = RcFailsafe(self)
            if trajectory_class is not None:
                self.trajectory: 'Optional[TrajectoryGenerator]' = trajectory_class(self)
            if heartbeat_handler:
                self.hearbeat_status_handler = HeartbeatStatusHandler(self)

        if "found" in outcomes:
            self.vision_trigger = VisionTrigger(self)

        if "target_position" in outcomes:
            self._target_position = TargetPosition(self)

        if "abort" in outcomes:
            # How to respond to a message sender
            self.message_senders.add(self.reusable_message_senders.find("abort"))
            self.handlers.add_handler("abort", self._on_abort)
        
        if "task_canceled" in outcomes:
            task_id = kwargs.get("task_id", -1)
            self.task_canceled_trigger = TaskCanceledTrigger(self, task_id)


    def _on_abort(self, message):
        return "abort"

    def on_entry(self, userdata):
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
        exit_outcome = self.on_exit(outcome, userdata)
        if self._is_performance_analysis_mode:
            self.message_queue.finish_processing()
        if exit_outcome is not None:
            return exit_outcome
        return outcome

    def execute2(self, userdata):
        """Method that runs every time we receive a message that hasn't caused
        a state transition
        """
        return None

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

    


    def _airlease_cleanup(self, _):
        """Send cleanup position update to air lease service
        """
        if self.air_lease_service is None:
            rospy.logdebug("_airlease_cleanup - air lease service not available")
            return

        if "position" not in self.message_data:
            rospy.logdebug("_airlease_cleanup - no position data available")
            return

        self.air_lease_service.send_cleanup(
            position=(
                self.message_data["position"].latitude,
                self.message_data["position"].longitude,
                self.message_data["position"].altitude
            )
        )


from dr_onboard_autonomy.states.components.trajectory import TrajectoryGenerator


