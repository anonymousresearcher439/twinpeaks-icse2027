import json

import rospy
from sensor_msgs.msg import NavSatFix, Imu

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.states import BaseState

from dr_onboard_autonomy.states.components import Gimbal


class UpdateSpacialData:
    """Component to update the position and attitude of the drone over time.
    """

    def __init__(self, state: BaseState):
        self.drone = state.drone
        state.message_senders.add(state.reusable_message_senders.find("position"))
        state.handlers.add_handler("position", self.on_position_message, is_active=False)

        state.message_senders.add(state.reusable_message_senders.find("imu"))
        state.handlers.add_handler("imu", self.on_imu_message, is_active=False)
    
    def on_position_message(self, message):
        navsat: NavSatFix = message['data']
        t = navsat.header.stamp.to_sec()
        lla = Lla(navsat.latitude, navsat.longitude, navsat.altitude)
        pvec = tuple(lla.to_pvector())
        self.drone.position_data.add(t, pvec)

    def on_imu_message(self, message):
        imu: Imu = message['data']
        t = imu.header.stamp.to_sec()
        q = imu.orientation
        q = q.x, q.y, q.z, q.w
        self.drone.attitude_data.add(t, q)
    
    def on_update_stare_position(self, message):
        message_payload = message["data"].payload.decode("utf-8")
        try:
            msg = json.loads(message_payload)
            stare_position = BriarLla(msg["stare_position"], is_amsl=True)
            self.gimbal_driver.stare_position = stare_position.ellipsoid.tup
            rospy.loginfo("Updating stare position to {}".format(stare_position.amsl.tup))
        except json.decoder.JSONDecodeError as e:
            rospy.logerr(f"Error decoding JSON: {e}")
            rospy.logerr("Message payload: " + message_payload)