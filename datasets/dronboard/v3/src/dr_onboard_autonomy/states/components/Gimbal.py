import math
from enum import Enum, auto
from typing import Tuple
from datetime import datetime

from dr_onboard_autonomy.logutils import DebounceLogger
import rospy

from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict, check_angle_with_threshold, distance_is_less_than
from dr_onboard_autonomy.gimbal_math import ChowdhuryMethod
from dr_onboard_autonomy.gimbal import GimbalCalculator2, GremsyGimbalCalculator
from dr_onboard_autonomy.gimbal.data_types import DroneData
from dr_onboard_autonomy.gimbal.data_types import Quaternion as DroneDataQuaternion
from dr_onboard_autonomy.gimbal.gimbal import MavlinkNode
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.models import IMU, TimeStampedLlaPosition, NedVelocity, Quaternion, LlaPosition
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.states.components.VideoMetadata import log_metadata
from dr_onboard_autonomy.briar_helpers import convert_Lla_to_tuple, ellipsoid_to_amsl
from droneresponse_mathtools import Lla

from dr_onboard_autonomy.states.components.Setpoint import Setpoint, SetpointType

GimbalCalculator = ChowdhuryMethod
GimbalCalculator = GimbalCalculator2
GimbalCalculator = GremsyGimbalCalculator

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

    def __init__(self, state: BaseState):
        self.dr_state = state
        self.drone = state.drone
        self.message_senders = state.message_senders
        self.reusable_message_senders = state.reusable_message_senders
        self.handlers = state.handlers
        self.trajectory = state.trajectory

        self.handlers.add_handler(Setpoint.TIMER_MESSAGE_NAME, self.on_setpoint_message, can_transition=False, priority=2)

        self.message_senders.add(RepeatTimer("gimbal", 0.1))
        self.handlers.add_handler("gimbal", self.on_gimbal_message)
        self.handlers.add_handler("gimbal", self.log_video_metadata)

        self.message_senders.add(self.reusable_message_senders.find("velocity"))
        self.handlers.add_handler("velocity", self.update_velocity)


        self.message_senders.add(self.reusable_message_senders.find("imu"))
        self.handlers.add_handler("imu", self.on_imu_message)

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position_message)

        self._mode = Gimbal.Mode.FIXED
        self._setpoint_value = (0, 0, 0)

        self.drone_data: DroneData = DroneData()
        self._enu_velocity = None
        self._gimbal_quaternion = None

        self.logger = DebounceLogger()
        self._fixed_direction_log_key = 5743
        self._stare_position_log_key = 5662
        self._stare_direction_log_key = 8528

    def start(self):
        self._mode = Gimbal.Mode.FIXED
        self._gimbal_quaternion = None

    def stop(self):
        self._mode = Gimbal.Mode.OFF
        self._setpoint_value = (0, 0, 0)
        self.drone.gimbal.set_neutral_attitude()
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
        self.logger.info(f"Gimbal: setting fixed direction to {p}", 1.0, self._fixed_direction_log_key)
    
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
        self.logger.info(f"Gimbal: setting stare position: {p}", 1.0, key=self._stare_position_log_key)

    @property
    def stare_direction(self):
        if self._mode != Gimbal.Mode.TRACK_DIRECTION:
            return None
        return self._setpoint_value

    @stare_direction.setter
    def stare_direction(self, p: Tuple[float, float, float]):
        self._setpoint_value = p
        self._mode = Gimbal.Mode.TRACK_DIRECTION
        self.logger.info(f"Gimbal: setting stare direction to {p}", 1.0, self._stare_direction_log_key)

    def on_imu_message(self, message):
        imu: IMU = message['data']
        new_attitude = imu.attitude
        self.drone_data = self.drone_data.update(attitude=DroneDataQuaternion(
            x=new_attitude.x,
            y=new_attitude.y,
            z=new_attitude.z,
            w=new_attitude.w,
        ))

    def on_position_message(self, message):
        pos: TimeStampedLlaPosition = message['data']
        new_pos = Lla(pos.latitude, pos.longitude, pos.altitude)
        self.drone_data = self.drone_data.update(position=new_pos)
    
    def on_setpoint_message(self, message):
        def track_position():
            if self.trajectory is None:
                return
            drone_pos = self.trajectory.setpoint_driver.lla
            if not drone_pos:
                return
            
            drone_pos = drone_pos.to_wgs84_ellipsoid()
            drone_pos = Lla(
                drone_pos.latitude,
                drone_pos.longitude,
                drone_pos.altitude
            )
            drone_data = self.drone_data.update(position=drone_pos)
            target = Lla(*self._setpoint_value)
            north, east, down = drone_pos.distance_ned(target)
            is_far_enough = not distance_is_less_than(north, east, down, 1.0) # meters
            is_not_vertical = not check_angle_with_threshold((north, east, down), threshold=2) # degrees
            if is_far_enough and is_not_vertical:
                self.trajectory.yaw = GimbalCalculator.find_aircraft_yaw(drone_data, target)

        if self._mode == Gimbal.Mode.TRACK_POSITION:
            track_position()

    def on_gimbal_message(self, message):
        def fixed():
            self.drone.gimbal.set_attitude(Quaternion(*self._gimbal_quaternion))
        def track_position():
            self._gimbal_quaternion = GimbalCalculator.find_track_position_attitude(self.drone_data, Lla(*self._setpoint_value))
            self.drone.gimbal.set_attitude(Quaternion(*self._gimbal_quaternion))
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
                "gimbal setpoint attitude i": x,
                "gimbal setpoint attitude j": y,
                "gimbal setpoint attitude k": z,
                "gimbal setpoint attitude w": w,
            })
        
        gimbal_attitude = self.drone.gimbal.get_attitude()
        if gimbal_attitude:
            data.update({
                "gimbal attitude i": gimbal_attitude.x,
                "gimbal attitude j": gimbal_attitude.y,
                "gimbal attitude k": gimbal_attitude.z,
                "gimbal attitude w": gimbal_attitude.w,
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
        linear_velocity: NedVelocity = message['data']

        self._enu_velocity = linear_velocity.east, linear_velocity.north, -linear_velocity.down

    @staticmethod
    def ellipsoid_lla_to_alt_amsl(lla: Lla):
        tup = convert_Lla_to_tuple(lla)
        tup = ellipsoid_to_amsl(tup)
        return tup[2]
