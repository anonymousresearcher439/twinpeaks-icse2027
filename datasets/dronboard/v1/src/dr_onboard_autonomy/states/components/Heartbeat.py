import copy
import json
import rospy

from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.message_senders import MQTTMessageSender
from typing import Union

class HeartbeatStatusHandler:
    """Component to modify flight based on heartbeat statuses

    Expects MQTT messages in the following format:
    message = 
    {
        "type" : "heartbeat_status"
        "data" : {
            "status" : <heartbeat status: 'continue', 'hover', or 'rtl'>
        }
    }
    """
    def __init__(self, state: BaseState):
        self._handlers = state.handlers
        self._state_drone = state.drone
        self._state_message_queue = state.message_queue
        self._state_message_senders = state.message_senders
        self._state_kwargs = state.args
        self._state_reusable_message_senders = state.reusable_message_senders
        self._state_trajectory = state.trajectory
        self._state_stop_message_senders = state.stop_message_senders
        self._state_start_message_senders = state.start_message_senders
        self._state_mqtt_client = state.local_mqtt_client

        _heartbeat_message_sender = MQTTMessageSender(
            "heartbeat_status", 
            "heartbeat_status", 
            self._state_mqtt_client
        )
        self._state_message_senders.add(_heartbeat_message_sender)
        self._state_message_senders.add(self._state_reusable_message_senders.find("heartbeat_status"))
        self._handlers.add_handler("heartbeat_status", self._on_heartbeat_status_message)

        '''
        HeartbeatHover imported here to avoid circular import reference with BaseState
        '''
        from dr_onboard_autonomy.states import HeartbeatHover
        hover_args = copy.copy(self._state_kwargs)
        hover_args["drone"] = self._state_drone
        hover_args["reusable_message_senders"] = self._state_reusable_message_senders
        hover_args["mqtt_client"] = self._state_mqtt_client

        self.heartbeat_hover = HeartbeatHover(**hover_args)


    def _on_heartbeat_status_message(self, message):
        data = message['data']
        self._state_drone.update_data("heartbeat_status", data["status"])

        if data["status"] == "hover":
            rospy.logdebug("HeartbeatStatusHandler - received hover status")
            return self._handle_hover()

        if data["status"] == "rtl":
            rospy.logdebug("HeartbeatStatusHandler - received rtl status")
            return self._handle_rtl()

        if data["status"] == "continue":
            rospy.logdebug("HeartbeatStatusHandler - received continue status")
            return self._handle_continue()                


    def _handle_hover(self):
        if self._run_hover_logic():
            rospy.loginfo("HeartbeatStatusHandler - running heartbeat hover logic")

            self._state_stop_message_senders()
            self._state_trajectory.pause()
            self._state_message_queue.queue.clear()

            hover_outcome = self.heartbeat_hover.execute(userdata=None)
            
            if hover_outcome != "succeeded_hover":
                return hover_outcome
            '''
            upon succeeded_hover outcome, restart the paused state's message senders and resume
            its trajectory
            '''
            self._state_trajectory.resume()
            self._state_start_message_senders()

            return

        return


    def _handle_rtl(self):
        rospy.loginfo("HeartbeatStatusHandler - running rtl logic - return rtl outcome")
        
        return "rtl"


    def _handle_continue(self):
        rospy.loginfo("HeartbeatStatusHandler - running continue logic")
        '''
        trigger no action as 'continue' status messages stream in one after another
        '''
        return

    
    def _run_hover_logic(self):
        rospy.logdebug(f"HeartbeatStatusHandler - _state_trajectory: {self._state_trajectory}")
        if self._state_trajectory is None:
            rospy.logwarn("HeartbeatStatusHandler - trajectory not implemented for current state - heartbeat hover will not be available")
        return(
            self._state_trajectory is not None
        )


