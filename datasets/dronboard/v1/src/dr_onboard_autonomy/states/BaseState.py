"""The BaseState module. All states extend the BaseState class in this module
"""
import time

from queue import Queue
from typing import Optional, Tuple, Type, Union

import rospy
import sensor_msgs.msg
import smach

from dr_onboard_autonomy.air_lease import AirLeaseService
from dr_onboard_autonomy.gimbal.data_types import Quaternion
from dr_onboard_autonomy.gimbal.gimbal_math2 import GimbalCalculator2
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import (RepeatTimer,
                                                 ReusableMessageSenders)
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states.components import (
    HeartbeatStatusHandler,
    RcFailsafe,
    UpdateSpacialData,
    VisionTrigger,
)

from .collections import MessageHandler, MessageQueue


class BaseState(smach.State):
    """All DR states extend this class. It has the execute method.
    """
    def __init__(
        self,
        air_lease_service: Optional[AirLeaseService]=None,
        drone: Optional[MAVROSDrone]=None,
        reusable_message_senders: Optional[ReusableMessageSenders]=None,
        mqtt_client: Optional[MQTTClient]=None,
        local_mqtt_client: Optional[MQTTClient]=None,
        heartbeat_handler: bool=True,
        trajectory_class: 'Type[TrajectoryGenerator]' =None,
        name: Optional[str]=None,
        **kwargs
    ):
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
        # self.message_queue = Queue()
        self.message_queue = MessageQueue()
        self.message_senders = set()
        self.handlers = MessageHandler()

        # To send the drone's status over MQTT  we need to make status messages and to make that we
        # need these message senders:
        def _get_data_handler(
            message_type: str
        ) -> Union[self._on_imu_message, self.on_data_update_message]:
            if message_type == "imu":
                return self._on_imu_message
            return self.on_data_update_message

        self.data_senders = {"position", "relative_altitude", "state", "battery", "imu"}
        for message_type in self.data_senders:
            message_sender = self.reusable_message_senders.find(message_type)
            self.message_senders.add(message_sender)
            self.handlers.add_handler(message_type, _get_data_handler(message_type))
        
        self._spacial_time_updater = UpdateSpacialData(self)

        self.send_data_timer = RepeatTimer("send_data_timer", 1)
        self.message_senders.add(self.send_data_timer)
        self.handlers.add_handler("send_data_timer", self.on_send_data)

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

        if "abort" in outcomes:
            # How to respond to a message sender
            self.message_senders.add(self.reusable_message_senders.find("abort"))
            self.handlers.add_handler("abort", self._on_abort)


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
        self.drone.update_data("state_name", self.name)

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
                self.stop_message_senders()
                exit_outcome = self.on_exit(outcome, userdata)
                if exit_outcome is not None:
                    return exit_outcome
                return outcome

            outcome = self.execute2(userdata)
            self.acknowledge_message(msg)
            if outcome is not None:
                self.stop_message_senders()
                exit_outcome = self.on_exit(outcome, userdata)
                if exit_outcome is not None:
                    return exit_outcome
                return outcome
            
            # How long did we spend processing the msg?
            # msg_processing_time = time.time() - msg_recv_time

            # log this in csv with a special prefix
            # line = f"@@@{time.time():.23f},{self.name},{type(self).__name__},{msg['type']},{msg_wait_time:.23f},{msg_processing_time:.23f},{len(self.message_queue)}"
            # rospy.logdebug(line)

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

    def on_data_update_message(self, _):
        """Update the status update data

        The drone reports it's status to the ground. This method updates the
        internal data that will eventually get sent.
        """
        is_data_available = all(
            [
                "battery" in self.message_data,
                "state" in self.message_data,
                "position" in self.message_data,
                "relative_altitude" in self.message_data,
            ]
        )

        if not is_data_available:
            return

        data = self.message_data
        position = {
            "latitude": data["position"].latitude,
            "longitude": data["position"].longitude,
            "altitude": data["relative_altitude"].data,
        }
        battery = {
            "voltage": data["battery"].voltage,
            "current": data["battery"].current,
            "level": data["battery"].percentage,
        }

        self.drone.update_data("location", position)
        self.drone.update_data("battery", battery)
        self.drone.update_data("status", data["state"].system_status)
        self.drone.update_data("mode", data["state"].mode)
        self.drone.update_data("state_name", self.name)
        self.drone.update_data("armed_state", data["state"].armed)

    def on_send_data(self, _):
        """Sends status update message to the ground
        """
        data = self.drone.data.to_dict()
        self.mqtt_client.publish("update_drone", data)

    def _on_imu_message(self, msg):
        """Save the gimbal attitude in the world frame of reference
        """
        drone_imu_msg: sensor_msgs.msg.Imu = msg['data']
        if self.drone.gimbal.attitude is not None:
            drone_attitude = Quaternion(
                drone_imu_msg.orientation.x,
                drone_imu_msg.orientation.y,
                drone_imu_msg.orientation.z,
                drone_imu_msg.orientation.w
            )
            self.drone.update_data(key="drone_attitude", val={
                "x": drone_attitude.x,
                "y": drone_attitude.y,
                "z": drone_attitude.z,
                "w": drone_attitude.w
            })

            heading = _compass_heading(
                x=drone_attitude.x,
                y=drone_attitude.y,
                z=drone_attitude.z,
                w=drone_attitude.w
            )
            self.drone.update_data(key="drone_heading", val=heading)

            gimbal_attitude_world = GimbalCalculator2.attitude_drone_to_world_frame(
                gimbal_attitude=Quaternion(
                    self.drone.gimbal.attitude[0],
                    self.drone.gimbal.attitude[1],
                    self.drone.gimbal.attitude[2],
                    self.drone.gimbal.attitude[3],
                ),
                drone_attitude=drone_attitude
            )
            self.drone.update_data(key="gimbal_attitude", val={
                "x": gimbal_attitude_world.x,
                "y": gimbal_attitude_world.y,
                "z": gimbal_attitude_world.z,
                "w": gimbal_attitude_world.w
            })


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

from tf.transformations import euler_from_quaternion
import math

def _compass_heading(x: float, y: float, z: float, w: float):
    try:
        q = x, y, z, w
        euler_angles = euler_from_quaternion(q)
        euler_angle = math.degrees(euler_angles[2])
        compass_angle = _enu_up_angle_to_compass(euler_angle)
        return compass_angle
    except:
        return None

def _enu_up_angle_to_compass(euler_angles_deg: float) -> float:
    x = 90 - euler_angles_deg
    return x % 360
