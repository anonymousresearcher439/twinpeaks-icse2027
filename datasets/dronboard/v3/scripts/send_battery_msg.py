#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import (
    BatteryState,
    NavSatFix,
    TimeReference,
)

import sys

battery_level = sys.argv[1]
battery_level = float(battery_level)

# Initialize the ROS node
rospy.init_node('battery_message_sender', anonymous=True)

# Create a publisher object
pub = rospy.Publisher('mavros/battery', BatteryState, queue_size=10)

# Wait a bit for the connection to be set up
rospy.sleep(1)

# Create and publish a message
battery = BatteryState = BatteryState()
"""
header: 
  seq: 3
  stamp: 
    secs: 1730839528
    nsecs: 548312958
  frame_id: ''
voltage: 12.600000381469727
temperature: 0.0
current: -0.0
charge: nan
capacity: nan
design_capacity: nan
percentage: 1.0
power_supply_status: 2
power_supply_health: 0
power_supply_technology: 0
present: True
cell_voltage: [12.600000381469727]
cell_temperature: []
location: "id0"
serial_number: ''
"""
battery.voltage = 12.6
battery.current = 4.0
battery.temperature = 0.0
battery.percentage = battery_level
battery.header.stamp = rospy.Time.now()
for _ in range(0, 10):
    pub.publish(battery)
    rospy.loginfo("Message published!")
    time.sleep(1.0)


# Optionally, keep the node running (e.g., if testing multiple messages)
# rospy.spin()