#!/usr/bin/env python

import rospy
import subprocess
import os

rospy.init_node("px4_simulator")

latitude = rospy.get_param("~latitude", "41.70573086523541")
longitude = rospy.get_param("~longitude", "-86.24421841999177")
altitude = rospy.get_param("~altitude", "100")
os.environ["PX4_HOME_LAT"] = str(latitude)
os.environ["PX4_HOME_LON"] = str(longitude)
os.environ["PX4_HOME_ALT"] = str(altitude)

headless_flag = rospy.get_param("~HEADLESS", 0)
if headless_flag:
    os.environ["HEADLESS"] = "1"
elif "HEADLESS" in os.environ:
    del os.environ["HEADLESS"]

px4_path = rospy.get_param("~px4_path", f"{os.environ['HOME']}/catkin_ws/src/px4")
os.chdir(px4_path)

subprocess.call(["make", "px4_sitl", "gazebo"])
