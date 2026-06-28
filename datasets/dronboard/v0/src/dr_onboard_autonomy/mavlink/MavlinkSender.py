
from dataclasses import dataclass, field
from threading import Lock
import rospy
import mavros.mavlink
from pymavlink.dialects.v20 import common as mavlink2

@dataclass
class MavlinkSender:
    system_id: int
    component_id: int
    mavlink_pub: rospy.Publisher
    protocol_handler: mavlink2.MAVLink = field(init=False, repr=False, hash=False, compare=False)
    lock: Lock = field(init=False, repr=False, hash=False, compare=False)
    
    def __post_init__(self):
        self.protocol_handler = mavlink2.MAVLink("", self.system_id, self.component_id)
        self.lock = Lock()

    def send(self, msg: mavlink2.MAVLink_message):
        with self.lock:            
            msg.pack(self.protocol_handler)
            rosmsg = mavros.mavlink.convert_to_rosmsg(msg)
            # there is a bug in convert_to_rosmsg where it doesn't set the seq number
            # this next line is a work around
            rosmsg.seq = self.protocol_handler.seq
            self.mavlink_pub.publish(rosmsg)
            self.protocol_handler.seq = (self.protocol_handler.seq + 1) % 256
