import json
from queue import Queue
from dr_onboard_autonomy.message_senders import RepeatTimer

import rospy
from .BaseState import BaseState


class ReceiveMission(BaseState):
    def __init__(self, return_channel=None, drone=None, reusable_message_senders=None, mqtt_client=None, **kwargs):
        """return_channel should be a Queue
        """
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(drone=drone, reusable_message_senders=reusable_message_senders, mqtt_client=mqtt_client, **kwargs)
        if return_channel is None:
            return_channel = Queue()
        self.return_channel = return_channel

        self.message_senders.add(reusable_message_senders.find("mission_spec"))
        self.handlers.add_handler("mission_spec", self.on_mission_spec)

        self.message_senders.add(RepeatTimer("log_info", 2.5))
        self.handlers.add_handler("log_info", self.on_log_info)
    
    def on_entry(self, userdata):
        self.mqtt_client.publish("new_drone", self.drone.data.to_dict())
    
    def on_log_info(self, _):
        rospy.loginfo("Awaiting new mission...")
    
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
            # TODO should we return "error" or wait for another mission?
            # return "error"