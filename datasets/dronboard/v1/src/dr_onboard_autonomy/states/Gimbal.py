import math
from dr_onboard_autonomy.message_senders import RepeatTimer
import rospy
from tf.transformations import (
    quaternion_from_euler,
)

from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict, EulerAnglesDict, QuaternionDict, convert_tuple_to_LlaDict
from dr_onboard_autonomy.states.components import Gimbal as GimbalComponent

from .BaseState import BaseState

_DEFAULT_POS = (0, 0, 0)
_DEFAULT_LLA_DICT = convert_tuple_to_LlaDict(_DEFAULT_POS)

_DEFAULT_QUATERNION_DICT = {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0,
    "w": 1.0,
}

_DEFAULT_EULER_ANGLES_DICT = {
    "roll_deg": 0,
    "pitch_deg": 0,
    "yaw_deg": 0,
}

class GimbalTestStarePoint(BaseState):
    """This state commands the gimbal. It sends stare position to the gimbal component.

    The stare_position is specified as a latitude, longitude, and altitude where the altitude is AMSL

    The 'succeeded' outcome will occur after `time` seconds
    The 'error' outcome will occure if the program is interrupted by the program exiting 
    """

    def __init__(self, data=None, time:float=5, stare_position:LlaDict=_DEFAULT_LLA_DICT, **kwargs):
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(**kwargs)

        self.gimbal = GimbalComponent(self)
        self.stare_position = BriarLla(stare_position, is_amsl=True)
        self.message_senders.add(RepeatTimer("done", time))
        self.handlers.add_handler("done", self.on_done)

    def on_entry(self, userdata):
        rospy.loginfo(f"Pointing gimbal at Lla{self.stare_position.amsl.tup})")
        self.gimbal.start()
        self.gimbal.stare_position = self.stare_position.ellipsoid.tup
    
    def on_done(self, _):
        self.gimbal.stop()
        return "succeeded"

class GimbalTestFixedQuaternion(BaseState):
    """This state commands the gimbal. It sends a quaternion to the gimbal component.

    The quaternion must be a unit quaternion specified with x, y, z, and w components

    The 'succeeded' outcome will occur after `time` seconds
    The 'error' outcome will occure if the program is interrupted by the program exiting 
    """
    def __init__(self, data=None, time:float=5, quaternion:QuaternionDict=_DEFAULT_QUATERNION_DICT, **kwargs):
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(**kwargs)

        self.gimbal = GimbalComponent(self)
        self.x = quaternion['x']
        self.y = quaternion['y']
        self.z = quaternion['z']
        self.w = quaternion['w']
        self.quaternion = self.x, self.y, self.z, self.w 
        self.message_senders.add(RepeatTimer("done", time))
        self.handlers.add_handler("done", self.on_done)

    def on_entry(self, userdata):
        tmp = [self.x, self.y, self.z, self.w]
        tmp = tuple([round(t, 3) for t in tmp])
        x, y, z, w = tmp        
        rospy.loginfo(f"Pointing gimbal at quaternion(x={x}, y={y}, z={z}, w={w})")

        self.gimbal.start()
        self.gimbal.fixed_direction = self.quaternion
    
    def on_done(self, _):
        self.gimbal.stop()
        return "succeeded"

class GimbalTestFixedEuler(BaseState):
    """This state commands the gimbal. It sends a quaternion to the gimbal component.

    The quaternion must be a unit quaternion specified with x, y, z, and w components

    The 'succeeded' outcome will occur after `time` seconds
    The 'error' outcome will occure if the program is interrupted by the program exiting 
    """
    def __init__(self, data=None, time:float=5, euler_angles:QuaternionDict=_DEFAULT_EULER_ANGLES_DICT, **kwargs):
        kwargs["outcomes"] = ["succeeded", "error"]
        super().__init__(**kwargs)

        self.gimbal = GimbalComponent(self)
        self.roll = euler_angles['roll_deg']
        self.pitch = euler_angles['pitch_deg']
        self.yaw = euler_angles['yaw_deg']
        euler_angles = self.roll, self.pitch, self.yaw
        euler_angles = [math.radians(a) for a in euler_angles]
        self.quaternion = quaternion_from_euler(*euler_angles)
        self.message_senders.add(RepeatTimer("done", time))
        self.handlers.add_handler("done", self.on_done)

    def on_entry(self, userdata):
        tmp = [self.roll, self.pitch, self.yaw]
        tmp = tuple([round(t, 3) for t in tmp])
        roll, pitch, yaw = tmp        
        rospy.loginfo(f"Pointing gimbal at euler_angles(roll={roll}, pitch={pitch}, yaw={yaw})")

        self.gimbal.start()
        self.gimbal.fixed_direction = self.quaternion
    
    def on_done(self, _):
        self.gimbal.stop()
        return "succeeded"
