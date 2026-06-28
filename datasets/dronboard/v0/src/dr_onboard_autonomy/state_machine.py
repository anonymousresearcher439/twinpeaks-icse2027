#!/usr/bin/env python3

import os
os.environ['MAVLINK20'] = "1"

import json
import queue
import rospy
import smach

from droneresponse_mathtools import geoid_height

from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import (
    ReusableMessageSenders,
    MQTTMessageSender,
    ReliableMessageSender,
)
from dr_onboard_autonomy.mqtt_client import MQTTClient
from dr_onboard_autonomy.states import default_transitions
from dr_onboard_autonomy.states import (
    AbortHover,
    Arm,
    ArmForce,
    BetterCircle,
    BetterHover,
    BetterPath,
    BriarCircle,
    BriarHover,
    BriarTravel,
    BriarWaypoint,
    Disarm,
    FlyWaypoints,
    Hover,
    Land,
    Preflight,
    Takeoff,
    HumanControl,
    Dropping,
    OnGround,
    PhasedCircle,
    PositionDrone,
    PossibleVictimDetected,
    ReceiveMission,
    ReturnHome,
    Searching,
    Standby,
    Tracking,
    VictimFound,
    Follow_with_cvTracking,
)


state_name_map = {
    "AbortHover": AbortHover,
    "Arm": Arm,
    "ArmForce": ArmForce,
    "BetterCircle": BetterCircle,
    "BetterHover": BetterHover,
    "BetterPath": BetterPath,
    "BriarCircle": BriarCircle,
    "BriarHover": BriarHover,
    "BriarTravel": BriarTravel,
    "BriarWaypoint": BriarWaypoint,
    "BriarWaypoint2": BriarWaypoint,
    "Disarm": Disarm,
    "Dropping": Dropping,
    "FlyWaypoints": FlyWaypoints,
    "Hover": Hover,
    "HumanControl": HumanControl,
    "Land": Land,
    "OnGround": OnGround,
    "PhasedCircle" : PhasedCircle,
    "PositionDrone": PositionDrone,
    "PossibleVictimDetected": PossibleVictimDetected,
    "Preflight": Preflight,
    "ReceiveMission": ReceiveMission,
    "ReturnHome": ReturnHome,
    "Searching": Searching,
    "Standby": Standby,
    "Takeoff": Takeoff,
    "Tracking": Tracking,
    "VictimFound": VictimFound,
    "Follow_with_cvTracking": Follow_with_cvTracking,
}


def run_state_machine(json_msg, args):
    mission_spec = json.loads(json_msg)
    rospy.loginfo(f"received mission-spec:\n\n'{mission_spec}'\n]n")

    state_machine = smach.StateMachine(outcomes=["failure", "mission_completed"])

    # extracting ned_waypoints form the data received from the drone
    rospy.logdebug("Printing waypoints received from the json file")

    altitude = args['home_altitude_offset']

    if "waypoints" in mission_spec:
        waypoints = read_waypoints(mission_spec["waypoints"], altitude)
        rospy.logdebug(f"here are the waypoints: {waypoints}")
        args["waypoints"] = waypoints

    rospy.logdebug("Printing the smach.StateMachine.add()")

    with state_machine:
        rospy.logdebug("Adding states to smach")
        rospy.logdebug("Adding states from mission_spec to smach")
        for i in mission_spec["states"]:
            state_name = i["name"]
            rospy.logdebug(f"Add {state_name} to smach")

            if state_name in ["MISSION_PREPARATION", "MissionPreparation"]:
                rospy.logdebug("Skipped MissionPreparation")
            else:
                if "args" in i:
                    state_args = build_args(args, i["args"])
                else:
                    state_args = args.copy()
                transition_spec = i["transitions"]
                add_state(state_name, transition_spec, state_args)

            rospy.logdebug("Done with " + state_name)
        rospy.logdebug("Done adding states from mission_spec")

        rospy.logdebug("Adding safety-related  states")
        safety_related_states = [
            "AbortHover",
            "HumanControl",
        ]
        for state_name in safety_related_states:
            add_state(state_name, [], args)

        rospy.logdebug("Finished adding states to smach")

    outcome = state_machine.execute()
    return outcome


def build_args(default_args, mission_spec_args):
    default_args = default_args.copy()
    mission_spec_args = mission_spec_args.copy()

    # for safety this won't let the mission_spec_args overwrite some values
    for critical_arg in ["drone", "reusable_message_senders", "mqtt_client", "uav_id"]:
        if critical_arg in mission_spec_args:
            del mission_spec_args[critical_arg]

    print(f"mission_spec_args = {mission_spec_args}")
    if "waypoints" in mission_spec_args:
        alt = default_args["home_altitude_offset"]
        # change how waypoints are specified
        waypoints = read_waypoints(mission_spec_args["waypoints"], alt)
        rospy.logdebug(f"here are the waypoints: {waypoints}")
        mission_spec_args["waypoints"] = waypoints

    default_args.update(mission_spec_args)
    return default_args


def read_waypoints(mission_spec_waypoints, home_altitude_offset):
    waypoints = []
    for i in mission_spec_waypoints:
        lla_position = (
            i["latitude"],
            i["longitude"],
            i["altitude"] + home_altitude_offset,
        )
        rospy.logdebug(f"waypoint: {lla_position}")
        waypoints.append(lla_position)
    return waypoints


def add_state(state_name, transition_spec, args):
    transitions = default_transitions(state_name)
    for transition in transition_spec:
        transitions[transition["condition"]] = transition["target"]

    Constructor = state_name_map[state_name]
    rospy.loginfo(f"Found Constructor for {state_name}: {Constructor}")
    rospy.loginfo(f"Adding {state_name} to smach with {transitions}")

    smach.StateMachine.add(state_name, Constructor(**args), transitions=transitions)


def mission_spec_topic(drone_id):
    topic = f"drone/{drone_id}/mission-spec"
    rospy.loginfo(f"Receiving mission from MQTT topic: '{topic}'")
    return topic


def read_uav_altitude_from_message_sender(reusable_message_senders):
    result_queue = queue.Queue()

    def recv_pos(message):
        pos = message["data"]
        # the pos is above the WGS-84 elipsoid.
        # we need to convert it to AMSL
        # when we tell mavros where to go, we must tell it in AMSL
        wgs84_to_amsl_offset = -geoid_height(pos.latitude, pos.longitude)
        alt = message["data"].altitude + wgs84_to_amsl_offset
        if alt is not None:
            result_queue.put(alt)

    reusable_message_senders.find("position").start(recv_pos)
    altitude = result_queue.get()
    reusable_message_senders.find("position").stop()

    rospy.loginfo(f"The UAV's altitude is {altitude}")
    return altitude

def build_default_args(drone, reusable_message_senders, mqtt_client, local_mqtt_client):
    altitude = read_uav_altitude_from_message_sender(reusable_message_senders)
    alt_str = format(altitude, ".3f")
    rospy.loginfo(f"Home altitude = {alt_str}")

    return {
        "drone": drone,
        "reusable_message_senders": reusable_message_senders,
        "mqtt_client": mqtt_client,
        "local_mqtt_client": local_mqtt_client,
        "uav_id": drone.uav_name,
        "hover_time": 10,
        "data": {},
        "home_altitude_offset": altitude,
    }


def main():
    rospy.init_node("OnboardPilot")

    uav_name = rospy.get_param("~uav_name", "unknown_uav")
    rospy.loginfo(f"The uav_name is '{uav_name}'")
    drone = MAVROSDrone(uav_name)
    drone.start()

    json = None

    reusable_message_senders = ReusableMessageSenders()
    reusable_message_senders.populate()

    mqtt_host = rospy.get_param("~mqtt_host", "127.0.0.1")
    rospy.loginfo(f"mqtt_host = {mqtt_host}")
    mqtt_client = MQTTClient(uav_name, mqtt_host)
    mqtt_client.connect()

    local_mqtt_host = rospy.get_param("~local_mqtt_host", "127.0.0.1")
    rospy.loginfo(f"local_mqtt_host = {local_mqtt_host}")
    local_mqtt_client = MQTTClient(uav_name, local_mqtt_host)
    local_mqtt_client.connect()

    # Sub to MQTT messages here
    reusable_message_senders.add(
        ReliableMessageSender(
            MQTTMessageSender("abort", "all-drones/abort", mqtt_client)
        )
    )
    reusable_message_senders.add(
        ReliableMessageSender(
            MQTTMessageSender("mission_spec", mission_spec_topic(uav_name), mqtt_client)
        )
    )


    reusable_message_senders.add(
        ReliableMessageSender(
            MQTTMessageSender("vision", 'vision', local_mqtt_client)
            )
        )
    

    reusable_message_senders.add(
        ReliableMessageSender(
            MQTTMessageSender("stop_following", 'stop_following', local_mqtt_client)
            )
        )

    args = build_default_args(drone, reusable_message_senders, mqtt_client, local_mqtt_client)

    recv_mission = ReceiveMission(**args)
    is_running = True
    while is_running:
        json_outcome = recv_mission.execute(None)
        if json_outcome == "succeeded":
            json = recv_mission.return_channel.get()
            rospy.loginfo(f"received a mission: '{json}'")

            outcome = run_state_machine(json, args)
            rospy.loginfo("FINAL : %s " % outcome)
            
            if outcome != "mission_completed":
                is_running = False
        else:
            rospy.logerr("did not receive mission spec")
            is_running = False


    # clean up
    reusable_message_senders.stop()
    if not rospy.is_shutdown():
        rospy.signal_shutdown("Done")
    rospy.spin()


if __name__ == "__main__":
    main()

