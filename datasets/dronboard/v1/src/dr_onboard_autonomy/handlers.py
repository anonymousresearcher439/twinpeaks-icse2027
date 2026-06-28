#!/usr/bin/env python3

from threading import Lock

import rospy
from mavros_msgs.msg import ExtendedState, State
from sensor_msgs.msg import BatteryState, NavSatFix
from std_msgs.msg import String


class NoTx:
    def __init__(self):
        pass

    def is_ready(self):
        return False


class TxCheck:
    def __init__(self, outcome):
        self.outcome = outcome

    def is_ready(self):
        return True


class BaseHandler:
    def __init__(self):
        pass

    def reg(self):
        rospy.loginfo("All Registered")

    def unreg(self):
        rospy.loginfo("All Unregistered")


class PosHandler(BaseHandler):
    def __init__(self):
        self.state_tx_checks["position"] = self.pos_tx_check
        self.pos = None

    def reg(self):
        self.position_sub = rospy.Subscriber(
            "/birdy0/mavros/global_position/global", NavSatFix, self.pos_callback
        )
        rospy.loginfo("Position Subscriber REGISTERED")
        super().reg()

    def pos_callback(self, pos):
        data = {"type": "position", "gps": pos}
        self.q.put(data)

    def unreg(self):
        self.position_sub.unregister()
        rospy.loginfo("Position Subscriber UNREGISTERED")
        super().unreg()

    def pos_tx_check(self, data):
        self.pos = data["gps"]
        return NoTx()


class BatteryHandler(BaseHandler):
    def __init__(self):
        self.state_tx_checks["battery"] = self.batt_tx_check
        self.batt = None

    def reg(self):
        self.battery_sub = rospy.Subscriber("/birdy0/mavros/battery", BatteryState)
        rospy.loginfo("Battery Subscriber REGISTERED")
        super().reg()

    def batt_callback(self, batt):
        data = {"type": "battery", "batt": batt}
        self.q.put(data)

    def unreg(self):
        self.battery_sub.unregister()
        rospy.loginfo("Battery Subscriber UNREGISTERED")
        super().unreg()

    def batt_tx_check(self, data):
        self.batt = data["batt"]
        return NoTx()


class CommandHandler(BaseHandler):
    def __init__(self):
        self.state_tx_checks["command"] = self.com_tx_check
        self.is_armed = None
        self.takeoff = None
        self.altitudereached = None
        self.done = None
        self.complete = None

    def reg(self):
        self.command_sub = rospy.Subscriber(
            "/birdy0/commands", String, self.command_callback
        )
        rospy.loginfo("Command Subscriber REGISTERED")
        super().reg()

    def command_callback(self, com):
        msg = {"type": "command", "data": com.data}
        self.q.put(msg)

    def unreg(self):
        self.command_sub.unregister()
        rospy.loginfo("Command Subscriber UNREGISTERED")
        super().unreg()

    def com_tx_check(self, msg):
        data = msg["data"]
        self.is_armed = data == "arm"
        self.takeoff = data == "takeoff"
        self.altitudereached = data == "hover"
        self.done = data == "land"
        self.complete = data == "finish"
        return NoTx()


class StateHandler(BaseHandler):
    def __init__(self):
        self.state_tx_checks["state"] = self.state_tx_check
        self.state = None

    def reg(self):
        self.state_sub = rospy.Subscriber(
            "/birdy0/mavros/state", State, self.state_callback
        )
        rospy.loginfo("State Subscriber REGISTERED")
        super().reg()

    def state_callback(self, state):
        msg = {"type": "state", "data": state}
        self.q.put(msg)

    def unreg(self):
        self.state_sub.unregister()
        rospy.loginfo("State Subscriber UNREGISTERED")
        super().unreg()

    def state_tx_check(self, msg):
        self.state = msg["data"]
        return NoTx()


class ExtendedStateHandler(BaseHandler):
    def __init__(self):
        self.state_tx_checks["ex_state"] = self.ex_state_tx_check
        self.ex_state = None

    def reg(self):
        self.state_sub = rospy.Subscriber(
            "/birdy0/mavros/extended_state", ExtendedState, self.ex_state_callback
        )
        rospy.loginfo("Extended State Subscriber REGISTERED")
        super().reg()

    def ex_state_callback(self, ex_sta):
        msg = {"type": "ex_state", "data": ex_sta}
        self.q.put(msg)

    def unreg(self):
        self.state_sub.unregister()
        rospy.loginfo("Extended State Subscriber UNREGISTERED")
        super().unreg()

    def ex_state_tx_check(self, msg):
        self.ex_state = msg["data"]
        return NoTx()


class ROSMessageHandler:
    def __init__(self, handler_name, state_data):
        self.handler_name
        self.state_data = state_data

    def handle_message(self, message):
        self.state_data[self.handler_name] = message["data"]


"""
class MQTTHandler(BaseHandler):
    def __init__(self, **kargs):
        self.state_tx_checks['mqtt'] = self.state_tx_check
        self.new_machine = None
        self.new_state_callback = kwargs['MQTTHandler']['']

    def reg(self):
        self.mqtt.register(...)
        rospy.loginfo('State Subscriber REGISTERED')
        super().reg()

    def state_callback(self, state):
        msg = {'type': "state", 'data': state}
        self.q.put(msg)

    def unreg(self):
        self.state_sub.unregister()
        rospy.loginfo('State Subscriber UNREGISTERED')
        super().unreg()

    def state_tx_check(self, msg):

        self.state = msg['data']
        return TxCheck('mqtt_smach')
"""
