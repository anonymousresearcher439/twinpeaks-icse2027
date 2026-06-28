import rospy
import json
import time
import random
from dr_onboard_autonomy.message_senders import RepeatTimer
import math
from tf.transformations import unit_vector
from numpy import ndarray
import numpy as np
import copy
from typing import List

from .BaseState import BaseState

from dr_onboard_autonomy.gimbal_math import ChowdhuryMethod


#This state follows an object or a person based on computer vision with tracking, currently keeps the gimbal to a fixed -45 degrees pitch.
class Follow_with_cvTracking(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = [
            "abort",
            "do_land",
            "done_following",
            "human_control",
            "rtl",
            "victim_near",
        ]
        super().__init__(**kwargs)

        self.setpoint_message_count = 0
        self.mode = "???"

        self.north=0.0
        self.east=0.0
        self.down=0.0

        self.x_scaled_coordinate=0 #scaled x, y coordinates
        self.y_scaled_coordinate=0

        self.gimbal=ChowdhuryMethod

        self.start=-1

        self.end=0

        self.message_senders.add(self.reusable_message_senders.find('stop_following'))
        self.handlers.add_handler("stop_following", self.on_stop_following)
        
        self.message_senders.add(self.reusable_message_senders.find('vision'))
        self.handlers.add_handler("vision", self.on_vision_message)

        self.message_senders.add(RepeatTimer("setpoint", 0.1))
        self.handlers.add_handler("setpoint", self.on_setpoint_message)

        self.handlers.add_handler("setpoint", self.trigger_offboard_mode)
        self.handlers.add_handler("state", self.update_mode)
        self.handlers.add_handler("state", self.trigger_offboard_mode)

        

    def on_vision_message(self, msg):

        vision_message = msg['data']
        #rospy.logwarn(f"on_vision_message got this vision_message: {vision_message}")
        json_str = vision_message.payload.decode("utf-8")
        vision_data = json.loads(json_str)
        #print("json message", vision_data)
        #rospy.logdebug(f"on_vision_message got this json string: {json_str}")
        #rospy.logdebug(f"on_vision_message got this vision_data: {vision_data}")

        timestamp = vision_data['timestamp']

        x = vision_data['x']
        y = vision_data['y']
        x_res = vision_data['x_res']
        y_res = vision_data['y_res']


        velocity_setpoint_coordinates=self.gimbal.create_vec3_in_camera_coordinates(x, y, x_res, y_res) #velocity set points and coordinates
        self.north=velocity_setpoint_coordinates[0]/10.0
        self.east=velocity_setpoint_coordinates[1]/10.0
        self.down=-velocity_setpoint_coordinates[2]/10.0


        print("North", self.north)
        print("East", self.east)
        print("Down", self.down)

        #scaled x,y coordinates

        """

        self.x_scaled_coordinate=x/x_res
        self.y_scaled_coordinate=y/y_res

        if (self.x_scaled_coordinate<=0.03 and self.y_scaled_coordinate<=0.03 ): #checking if the object is close to the coordinate center, need to test with more experimental values

            return "do_land"
   
        """

    
    def on_stop_following(self, message):

        print("Received stop message")

        return "done_following"



    def on_setpoint_message(self, message):

        if (self.start==-1):

            self.start=time.time()


        #current_waypoint = (37, -118, 50)
        #v=self.north, self.east, self.down
        v=(self.north, self.east, self.down)

        #self.drone.send_setpoint(lla=current_waypoint)
        self.drone.send_setpoint(ned_velocity=v)
        self.setpoint_message_count += 1
        self.end=time.time()

        difference=self.end - self.start

        if difference>600.0:

            print("done......................................................................................")

            return "done_following"


    
    def update_mode(self, message):
        if message["type"] == "state" and message["data"] is not None:
            self.mode = message["data"].mode

    def trigger_offboard_mode(self, message):
        if self.setpoint_message_count < 20:
            return

        if self.mode != "OFFBOARD":
            if not self.drone.set_mode("OFFBOARD"):
                return "error"
            else:
                self.mode = "OFFBOARD"



