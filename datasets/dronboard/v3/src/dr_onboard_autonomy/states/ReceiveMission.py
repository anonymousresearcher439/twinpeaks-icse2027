import json
from queue import Queue
from typing import Optional

from dr_onboard_autonomy.models.config import CameraConfig
import rospy
from .BaseState import BaseState

from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


@register_state(default_transitions={"error": "failure"})
class ReceiveMission(BaseState):
    def __init__(self,
            return_channel=None,
            drone=None,
            reusable_message_senders=None,
            mqtt_client=None,
            camera_config:Optional[CameraConfig]=None,
            previous_outcome:Optional[str]=None,
            **kwargs
        ):
        """return_channel should be a Queue
        """
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(drone=drone, reusable_message_senders=reusable_message_senders, mqtt_client=mqtt_client, camera_config=camera_config, **kwargs)
        if return_channel is None:
            return_channel = Queue()
        self.return_channel = return_channel
        self.camera_config = camera_config

        self.message_senders.add(reusable_message_senders.find("mission_spec"))
        self.handlers.add_handler("mission_spec", self.on_mission_spec)

        self.message_senders.add(RepeatTimer("log_info", 2.5))
        self.handlers.add_handler("log_info", self.on_log_info)

        self.message_senders.add(RepeatTimer("new_drone_message", 0.5))
        self.count = 0
        self.handlers.add_handler("new_drone_message", self.send_new_drone_message)
        self.previous_outcome = previous_outcome
    
    def on_entry(self, userdata):
        pass
        # self.mqtt_client.publish("new_drone", self.drone.data.to_dict())
    
    def send_new_drone_message(self, _msg):
        if self.count % 4 == 0:
            msg = self.drone.data.to_dict()
            if self.camera_config:
                msg['camera'] = {
                    "horizontal_fov": self.camera_config.horizontal_fov,
                    "vertical_fov": self.camera_config.vertical_fov
                }
            self.mqtt_client.publish("new_drone", msg)
        self.count += 1
    
    def on_log_info(self, _):
        rospy.loginfo("Awaiting new mission...")
        self.send_ready_status()
    
    def on_mission_spec(self, message):
        rospy.logdebug(f"in on_mission_spec {str(message)}")
        mission_message = message['data']
        try:
            m = mission_message.payload.decode("utf-8")
            _ = json.loads(m) # if something is wrong, let's tell the user
            self.return_channel.put(m)
            return "succeeded"
        except json.decoder.JSONDecodeError as json_error:
            uav_id = self.drone.uav_name
            topic = f"drone/{uav_id}/error"
            data = f"Error trying to receive a mission. Something was wrong with the JSON message. Here is the error message: '{str(json_error)}'. Here is the message we received: '{json_error.doc}'"
            self.mqtt_client.publish(topic, data)
            rospy.logerr(data)
    
    def send_ready_status(self):
        uav_id = self.drone.uav_name
        topic = f"drone/{uav_id}/mission_status"
        msg = {
            "status": "ready",
            "uavid": uav_id,
            "time": self.mqtt_client.timestamp()
        }
        if self.previous_outcome:
            msg["previous_outcome"] = self.previous_outcome
        self.mqtt_client.publish(topic, msg)


@register_state(default_transitions=STANDARD_TRANSITIONS_FLYING)
class ReceiveMissionAirborne(BaseState):
    def __init__(self, return_channel=None, **kwargs):
        """return_channel should be a Queue
        """
        # We need to add some more outcomes to make sure we respond
        # appropriately in case the remote pilot takes action        
        unwanted_outcomes = kwargs.get("unwanted_outcomes", set())
        kwargs['outcomes'] = self._process_outcomes(kwargs, ["succeeded"], STANDARD_TRANSITIONS_FLYING.keys(), unwanted_outcomes)
        
        kwargs.update({
            'outcomes': list(all_outcomes),
            'trajectory_class': HoldingTrajectory,
        })
        super().__init__(**kwargs)
        if return_channel is None:
            return_channel = Queue()
        self.return_channel = return_channel

        self.message_senders.add(self.reusable_message_senders.find("mission_spec"))
        self.handlers.add_handler("mission_spec", self.on_mission_spec)

        self.message_senders.add(RepeatTimer("log_info", 2.5))
        self.handlers.add_handler("log_info", self.on_log_info)
    
    def on_entry(self, userdata):
        self.trajectory.start()
        self.send_ready_status()
    
    on_log_info = ReceiveMission.on_log_info
    on_mission_spec = ReceiveMission.on_mission_spec
    send_ready_status = ReceiveMission.send_ready_status

