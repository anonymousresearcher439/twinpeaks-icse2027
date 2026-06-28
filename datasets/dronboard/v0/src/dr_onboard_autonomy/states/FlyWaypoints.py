import rospy
from droneresponse_mathtools import Lla, geoid_height

from dr_onboard_autonomy.message_senders import RepeatTimer

from .BaseState import BaseState

from dr_onboard_autonomy.gimbal_math import ChowdhuryMethod

import tf
from threading import Thread
from dr_onboard_autonomy.gimbal import MavlinkNode
import math


class FlyWaypoints(BaseState):
    def __init__(self, data=None, waypoints=[], **kwargs):
        outcomes = []
        if "outcomes" in kwargs:
            outcomes = kwargs["outcomes"]

        for other_outcome in ["succeeded_waypoints", "error", "human_control", "abort", "found"]:

            if other_outcome not in outcomes:
                outcomes.append(other_outcome)
        kwargs["outcomes"] = outcomes
        super().__init__(**kwargs)

        self.data = data
        self.ned_waypoints = waypoints
        self.home_pos = None
        self.is_mode_changed = False
        self.setpoint_message_count = 0
        self.pos_message_count = 0
        self.mode = "???"

        self.index = 0

        self.gimbal_manager = MavlinkNode(1, 1)

        self.drone_lat=0
        self.drone_long=0
        self.drone_altitide=0

        self.drone_orientation_x=0
        self.drone_orientation_y=0
        self.drone_orientation_z=0
        self.drone_orientation_w=0

        self.compass_heading=0
        self.gimbal=ChowdhuryMethod

        self.target_gps_location = kwargs['target_gps_location']
        self.gps_target_lat=self.target_gps_location['latitude']
        self.gps_target_long=self.target_gps_location['longitude']
        self.gps_target_alt=self.target_gps_location['altitude']

        self.mission_modes = kwargs['mission_mode']
        self.mission_mode=self.mission_modes['mode']

        if "found" in outcomes:

            self.message_senders.add(self.reusable_message_senders.find('vision'))
            self.handlers.add_handler("vision", self.on_vision_message)

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.message_senders.add(self.reusable_message_senders.find("state"))
        self.message_senders.add(self.reusable_message_senders.find("imu"))
        self.message_senders.add(self.reusable_message_senders.find("compass_hdg"))

        self.message_senders.add(RepeatTimer("setpoint", 0.1))

        self.handlers.add_handler("imu", self.on_imu_message)
        self.handlers.add_handler("compass_hdg", self.on_compass_heading_message)
        self.handlers.add_handler("position", self.on_position_message)
        self.handlers.add_handler("setpoint", self.on_setpoint_message)
        self.handlers.add_handler("setpoint", self.trigger_offboard_mode)
        self.handlers.add_handler("state", self.trigger_offboard_mode)
        self.handlers.add_handler("state", self.update_mode)

    def on_entry(self, userdata):
        self.index = 0
        home = self.data["arm_position"]
        #print(home)
        self.home_pos = Lla(home.latitude, home.longitude, home.altitude)
        self.drone.gimbal.take_control(self.gimbal_manager)
        if self.mission_mode==2:
            self.drone.gimbal_rotation(0, -90, 0) #setting the gimbal to look down to detect objects as it gets the best view from the top
        

    def on_vision_message(self, message):

        if self.mission_mode==2:  #if mode is 2 then vision component is on
            return "found"
        else:
            pass

    def on_position_message(self, message):
        self.pos_message_count += 1
        pos = message["data"]
        current_pos = Lla(pos.latitude, pos.longitude, pos.altitude)
        waypoint_lat, waypoint_lon, waypoint_alt = self.ned_waypoints[self.index]
        waypoint_alt = waypoint_alt + geoid_height(waypoint_lat, waypoint_lon)
        target = Lla(waypoint_lat, waypoint_lon, waypoint_alt)

        self.drone_lat=current_pos[0]
        self.drone_long=current_pos[1]
        self.drone_altitide=current_pos[2]

        #print (self.drone_altitide)

        distance = current_pos.distance(target)
        if distance < 1.0:
            self.index += 1
            rospy.loginfo("moving to next waypoint")
            if len(self.ned_waypoints) == self.index:
                return "succeeded_waypoints"
        if self.pos_message_count % 120 == 0:
            rospy.loginfo(
                f"waypoint: {self.index:3} of {len(self.ned_waypoints): 3} | distance: {distance:4.2f} meters | current: {current_pos} | target: {target}"
            )
    def on_imu_message(self, message):

        drone_orientations = message["data"]
        self.drone_orientation_x=drone_orientations.orientation.x
        self.drone_orientation_y=drone_orientations.orientation.y
        self.drone_orientation_z=drone_orientations.orientation.z
        self.drone_orientation_w=drone_orientations.orientation.w

    def on_compass_heading_message(self, message):

        self.compass_heading= message["data"]


    def on_setpoint_message(self, message):

        current_waypoint = self.ned_waypoints[self.index]
        self.drone.send_setpoint(lla=current_waypoint)
        self.setpoint_message_count += 1


        if self.mission_mode==2:

            if self.setpoint_message_count%5==0:

                self.drone.gimbal_rotation(0, -45, 0)


        if self.mission_mode==1:

            if self.setpoint_message_count%5==0:

                gimbal_rotation=self.gimbal.gimbal_angles(self.drone_lat, self.drone_long, self.drone_altitide, self.drone_orientation_x, self.drone_orientation_y, self.drone_orientation_z, self.drone_orientation_w, self.gps_target_lat, self.gps_target_long, self.gps_target_alt, self.compass_heading)
                gimbal_roll=gimbal_rotation[0]
                gimbal_pitch=gimbal_rotation[1]
                gimbal_yaw=gimbal_rotation[2]
                quaternion = tf.transformations.quaternion_from_euler(math.radians(gimbal_roll), math.radians(gimbal_pitch), math.radians(gimbal_yaw))
                self.drone.gimbal.set_attitude(quaternion, self.gimbal_manager)
                #self.drone.gimbal_rotation(gimbal_roll, gimbal_pitch, gimbal_yaw)

    def trigger_offboard_mode(self, message):
        if self.setpoint_message_count < 20:
            return

        if self.mode != "OFFBOARD":
            if not self.drone.set_mode("OFFBOARD"):
                return "error"
            else:
                self.mode = "OFFBOARD"

    def update_mode(self, message):
        if message["type"] == "state" and message["data"] is not None:
            self.mode = message["data"].mode