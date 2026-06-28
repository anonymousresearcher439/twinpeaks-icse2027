import time

from enum import Enum, auto
from typing import Tuple
from datetime import datetime

from geometry_msgs.msg import TwistStamped
from tf.transformations import (
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_conjugate,
    quaternion_matrix,
    quaternion_about_axis,
    quaternion_multiply,
    unit_vector,
    euler_from_quaternion,
)


from dr_onboard_autonomy.gimbal_math import ChowdhuryMethod
from dr_onboard_autonomy.gimbal import GimbalCalculator2, SDHX10GimbalCalculator
from dr_onboard_autonomy.gimbal.data_types import DroneData, Quaternion
from dr_onboard_autonomy.gimbal.gimbal import MavlinkNode
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components.VideoMetadata import log_metadata
from dr_onboard_autonomy.briar_helpers import convert_Lla_to_tuple, ellipsoid_to_amsl
from droneresponse_mathtools import Lla

import rospy

GimbalCalculator = ChowdhuryMethod
GimbalCalculator = GimbalCalculator2
GimbalCalculator = SDHX10GimbalCalculator

class Gimbal:
    """This component gives states the ability to control the gimbal
    
    There are three ways to command the gimbal:
    1. You can set a fixed direction given as a quaternion
    2. You can set a stare position given as a latitude longitude and altitude


    """
    class Mode(Enum):
        FIXED = auto()
        TRACK_POSITION = auto()
        TRACK_DIRECTION = auto()
        OFF = auto()

    def __init__(self, state: BaseState, gimbal_manager:MavlinkNode=None):
        self.dr_state = state
        self.drone: MAVROSDrone = state.drone
        self.message_senders = state.message_senders
        self.reusable_message_senders = state.reusable_message_senders
        self.handlers = state.handlers
        self.trajectory = state.trajectory
        
        self.message_senders.add(RepeatTimer("gimbal", 0.1))
        self.handlers.add_handler("gimbal", self.on_gimbal_message)
        self.handlers.add_handler("gimbal", self.log_video_metadata)

        self.message_senders.add(self.reusable_message_senders.find("velocity"))
        self.handlers.add_handler("velocity", self.update_velocity)


        self.message_senders.add(self.reusable_message_senders.find("imu"))
        self.handlers.add_handler("imu", self.on_imu_message)

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position_message)

        if gimbal_manager is None:
            gimbal_manager = MavlinkNode(self.drone.system_id, self.drone.fc_component_id)
        self._gimbal_manager = gimbal_manager
        self._mode = Gimbal.Mode.FIXED
        self._setpoint_value = (0, 0, 0)

        self.drone_data: DroneData = DroneData()
        self._enu_velocity = None
        self._gimbal_quaternion = None

    def start(self):
        self._mode = Gimbal.Mode.FIXED
        self._gimbal_quaternion = None
        self.drone.gimbal.take_control(self._gimbal_manager)
        # maybe we should give secondary control to QGroundControl? 
        #self.drone.gimbal.take_control(self._gimbal_manager, secondary_controller=MavlinkNode(255, 0))

    def stop(self):
        self._mode = Gimbal.Mode.OFF
        self._setpoint_value = (0, 0, 0)
        self.drone.gimbal.reset_attitude(self._gimbal_manager)
        neutral_quaternion = (0.0, 0.0, 0.0, 1.0)
        self.drone.gimbal.attitude_data.add(time.time(), neutral_quaternion)
        self._gimbal_quaternion = None

    
    @property
    def fixed_direction(self):
        if self._mode != Gimbal.Mode.FIXED:
            return None
        return self._gimbal_quaternion

    @fixed_direction.setter
    def fixed_direction(self, p: Tuple[float, float, float, float]):
        """ quaternion is interpreted as (X, Y, Z, W) 
        """
        self._gimbal_quaternion = p
        self._mode = Gimbal.Mode.FIXED
    
    @property
    def stare_position(self):
        if self._mode != Gimbal.Mode.TRACK_POSITION:
            return None
        return self._setpoint_value

    @stare_position.setter
    def stare_position(self, p: Tuple[float, float, float]):
        """look at the given latitude longitude altitude.
        Altitude is ellipsoidal.
        """
        self._setpoint_value = p
        self._mode = Gimbal.Mode.TRACK_POSITION
    
    @property
    def stare_direction(self):
        if self._mode != Gimbal.Mode.TRACK_DIRECTION:
            return None
        return self._setpoint_value

    @stare_direction.setter
    def stare_direction(self, p: Tuple[float, float, float]):
        self._setpoint_value = p
        self._mode = Gimbal.Mode.TRACK_DIRECTION
    
    def on_imu_message(self, message):
        q = message['data'].orientation
        new_attitude = Quaternion(q.x, q.y, q.z, q.w)
        self.drone_data = self.drone_data.update(attitude=new_attitude)

    def on_position_message(self, message):
        pos = message['data']
        new_pos = Lla(pos.latitude, pos.longitude, pos.altitude)
        self.drone_data = self.drone_data.update(position=new_pos)
    
    def on_gimbal_message(self, message):
        def fixed():
            self.drone.gimbal.set_attitude(self._gimbal_quaternion, self._gimbal_manager)
            self.drone.gimbal.attitude_data.add(time.time(), self._gimbal_quaternion)
        def track_position():
            self._gimbal_quaternion = GimbalCalculator.find_track_position_attitude(self.drone_data, Lla(*self._setpoint_value))
            self.drone.gimbal.set_attitude(self._gimbal_quaternion, self._gimbal_manager)
            self.drone.gimbal.attitude_data.add(time.time(), self._gimbal_quaternion)
            if self.trajectory is None:
                return
            self.trajectory.yaw = GimbalCalculator.find_aircraft_yaw(self.drone_data, Lla(*self._setpoint_value))
        def track_direction():
            pass
        def off():
            pass

        function_table = {
            Gimbal.Mode.FIXED: fixed,
            Gimbal.Mode.TRACK_POSITION: track_position,
            Gimbal.Mode.TRACK_DIRECTION: track_direction,
            Gimbal.Mode.OFF: off,
        }
        send_gimbal_cmd_func = function_table[self._mode]
        send_gimbal_cmd_func()
    
    def log_video_metadata(self, message):
        time_stamp = datetime.now().isoformat()
        drone_alt_amsl = self.ellipsoid_lla_to_alt_amsl(self.drone_data.position)
        data = {
            "timestamp": time_stamp,
            "drone latitude": self.drone_data.position.latitude,
            "drone longitude": self.drone_data.position.longitude,
            "drone altitude": drone_alt_amsl,
            "drone attitude i": self.drone_data.attitude.x,
            "drone attitude j": self.drone_data.attitude.y,
            "drone attitude k": self.drone_data.attitude.z,
            "drone attitude w": self.drone_data.attitude.w,
        }

        if self._enu_velocity is not None:
            e, n, u = self._enu_velocity
            data.update({
                "drone velocity E": e,
                "drone velocity N": n,
                "drone velocity U": u,
            })
        
        if self._gimbal_quaternion is not None:
            x, y, z, w = self._gimbal_quaternion
            data.update({
                "gimbal attitude i": x,
                "gimbal attitude j": y,
                "gimbal attitude k": z,
                "gimbal attitude w": w,
            })
        
        if self._mode == Gimbal.Mode.TRACK_POSITION:
            lat, lon, alt = ellipsoid_to_amsl(self._setpoint_value)
            data.update({
                "stare point latitude": lat,
                "stare point longitude": lon,
                "stare point altitude": alt,
            })
        log_metadata(data)
        
    
    def update_velocity(self, message):
        twist_stamped: TwistStamped = message['data']
        x = twist_stamped.twist.linear.x
        y = twist_stamped.twist.linear.y
        z = twist_stamped.twist.linear.z

        self._enu_velocity = float(x), float(y), float(z)

    @staticmethod
    def ellipsoid_lla_to_alt_amsl(lla: Lla):
        tup = convert_Lla_to_tuple(lla)
        tup = ellipsoid_to_amsl(tup)
        return tup[2]
