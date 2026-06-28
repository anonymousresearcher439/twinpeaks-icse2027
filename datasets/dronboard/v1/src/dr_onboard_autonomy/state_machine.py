#!/usr/bin/env python3

import os
from threading import Event

from dr_onboard_autonomy.air_lease import AirLeaseService

os.environ['MAVLINK20'] = "1"

import json
import queue

import rospy
import smach
from mavros_msgs.msg import State
from droneresponse_mathtools import geoid_height

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.mavros_layer import MAVROSDrone
from dr_onboard_autonomy.message_senders import (
    HeartbeatMessageSender,
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
    CircleVisionTarget,
    Disarm,
    FlyWaypoints,
    GimbalTestFixedEuler,
    GimbalTestFixedQuaternion,
    GimbalTestStarePoint,
    HeartbeatHover,
    Hover,
    Land,
    Preflight,
    UnstableTakeoff,
    Takeoff,
    HumanControl,
    Dropping,
    OnGround,
    PhasedCircle,
    PositionDrone,
    PossibleVictimDetected,
    ReadPosition,
    ReceiveMission,
    ReturnHome,
    Rtl,
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
    "BriarWaypoint3": BriarWaypoint,
    "CircleVisionTarget": CircleVisionTarget,
    "Disarm": Disarm,
    "Dropping": Dropping,
    "FlyWaypoints": FlyWaypoints,
    "GimbalTestFixedEuler": GimbalTestFixedEuler,
    "GimbalTestFixedQuaternion": GimbalTestFixedQuaternion,
    "GimbalTestStarePoint": GimbalTestStarePoint,
    "HeartbeatHover": HeartbeatHover,
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
    "Rtl": Rtl,
    "Searching": Searching,
    "Standby": Standby,
    "UnstableTakeoff": UnstableTakeoff,
    "Takeoff": Takeoff,
    "Tracking": Tracking,
    "VictimFound": VictimFound,
    "Follow_with_cvTracking": Follow_with_cvTracking,
}


def _build_mission(json_msg, args):
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
            
            class_name = None
            if "class" in i:
                class_name = i["class"]
            rospy.logdebug(f"Add {state_name} to smach")

            if state_name in ["MISSION_PREPARATION", "MissionPreparation"]:
                rospy.logdebug("Skipped MissionPreparation")
            else:
                if "args" in i:
                    state_args = build_args(args, i["args"])
                else:
                    state_args = args.copy()
                transition_spec = i["transitions"]
                add_state(state_name, transition_spec, state_args, class_name)

            rospy.logdebug("Done with " + state_name)
        rospy.logdebug("Done adding states from mission_spec")

        rospy.logdebug("Adding safety-related  states")
        safety_related_states = [
            "AbortHover",
            "HumanControl",
            "Rtl"
        ]
        for state_name in safety_related_states:
            add_state(state_name, [], args)

        rospy.logdebug("Finished adding states to smach")
    return state_machine

def run_state_machine(json_msg, args):
    state_machine = _build_mission(json_msg, args)
    outcome = state_machine.execute()
    return outcome

def verify_mission(json_msg, args):
    """
    Return True of the mission is ok
    Return exception otherwise
    """
    state_machine = _build_mission(json_msg, args)
    try:
        state_machine.check_consistency()
        return True
    except Exception as e:
        return e

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
        i['altitude'] = i['altitude'] + home_altitude_offset
        lla_position = (
            i["latitude"],
            i["longitude"],
            i["altitude"],
        )
        rospy.logdebug(f"waypoint: {lla_position}")
        waypoints.append(i)
    return waypoints


def add_state(state_name, transition_spec, args, class_name:str = None):
    if class_name is None:
        class_name = state_name
    transitions = default_transitions(class_name)
    for transition in transition_spec:
        transitions[transition["condition"]] = transition["target"]
    
    all_outcomes = set()
    for outcome in transitions:
        all_outcomes.add(outcome)
    
    if "outcomes" in args:
        for outcome in args['outcomes']:
            all_outcomes.add(outcome)
    args['outcomes'] = list(all_outcomes)

    Constructor = state_name_map[class_name]
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
    result = {
        "air_lease_service" : AirLeaseService(uav_id=drone.uav_name, mqtt_client=mqtt_client),
        "drone": drone,
        "reusable_message_senders": reusable_message_senders,
        "mqtt_client": mqtt_client,
        "local_mqtt_client": local_mqtt_client,
        "uav_id": drone.uav_name,
        "data": {},
    }
    return_channel = queue.Queue()
    position_reader = ReadPosition(return_channel=return_channel, **result)
    outcome = position_reader.execute(None)
    if outcome != "succeeded":
        rospy.loginfo("could not read drone position")
        rospy.signal_shutdown("could not read drone position")
        raise Exception(f"could not read position. position_reader outcome: '{outcome}'")
        
    pos: BriarLla = return_channel.get()
    lat, lon, alt = pos.amsl.lla.lat, pos.amsl.lla.lon, round(pos.amsl.lla.altitude, 2)
    rospy.loginfo(f"Position: Lla{lat, lon, alt}")
    altitude = pos.amsl.lla.altitude
    alt_str = format(altitude, ".2f")
    rospy.loginfo(f"Home Altitude (AMSL): {alt_str}")
    result.update({
        "home_altitude_offset": altitude
    })
    return result


def _await_mavros(reusable_message_senders: ReusableMessageSenders):
    rospy.loginfo("Waiting for mavros...")
    
    state_msg_sender = reusable_message_senders.find("state")
    shutdown_msg_sender = reusable_message_senders.find("shutdown")

    is_mavros_up = False
    
    callback_event = Event()
    def on_msg(msg):
        nonlocal is_mavros_up
        is_mavros_up = msg['type'] == 'state'
        callback_event.set()

    state_msg_sender.start(on_msg)
    shutdown_msg_sender.start(on_msg)

    callback_event.wait()

    if not is_mavros_up:
        raise Exception("mavros not detected")
    
    rospy.loginfo("mavros detected.")
    state_msg_sender.stop()
    shutdown_msg_sender.stop()

    
def _await_px4(reusable_message_senders: ReusableMessageSenders):
    rospy.loginfo("Waiting for px4...")
    
    state_msg_sender = reusable_message_senders.find("state")
    shutdown_msg_sender = reusable_message_senders.find("shutdown")

    is_px4_up = False
    
    callback_event = Event()
    def on_msg(msg):
        nonlocal is_px4_up
        if msg['type'] == 'shutdown':
            rospy.loginfo("received shutdown message while waiting for px4")
            is_px4_up = False
            callback_event.set()
            return

        state: State = msg['data']
        is_px4_up = msg['data'].connected
        rospy.logdebug(f"_await_px4 callback: state: {state}")
        if is_px4_up:
            callback_event.set()

    state_msg_sender.start(on_msg)
    shutdown_msg_sender.start(on_msg)

    callback_event.wait()

    if not is_px4_up:
        raise Exception("px4 not connected")

    rospy.loginfo("px4 connected")
    state_msg_sender.stop()
    shutdown_msg_sender.stop()


def main():
    # TODO change debug to info
    rospy.init_node("OnboardPilot", log_level=rospy.INFO)

    rospy.logdebug(f"@@@timestamp,state_name,state_class,message_type,message_wait_time_seconds,message_processing_time_seconds,message_queue_size")


    uav_name = rospy.get_param("~uav_name", "unknown_uav")
    rospy.loginfo(f"The uav_name is '{uav_name}'")

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
            MQTTMessageSender('airlease_status', f"drone/{uav_name}/airlease/status", mqtt_client)
        )
    )
    reusable_message_senders.add(
        ReliableMessageSender(
            MQTTMessageSender("mission_spec", mission_spec_topic(uav_name), mqtt_client)
        )
    )
    reusable_message_senders.add(
        ReliableMessageSender(
            MQTTMessageSender("update_stare_position", f"drone/{uav_name}/stare-position", mqtt_client)
        )
    )
    
    reusable_message_senders.add(
        MQTTMessageSender("vision", 'vision', local_mqtt_client)
    )
    reusable_message_senders.add(
        ReliableMessageSender(
            MQTTMessageSender("stop_following", 'stop_following', local_mqtt_client)
        )
    )

    _await_mavros(reusable_message_senders)
    _await_px4(reusable_message_senders)

    drone = MAVROSDrone(uav_name)
    drone.start()

    json = None

    args = build_default_args(drone, reusable_message_senders, mqtt_client, local_mqtt_client)

    recv_mission = ReceiveMission(**args)
    is_running = True
    while is_running:
        json_outcome = recv_mission.execute(None)
        if json_outcome == "succeeded":
            json = recv_mission.return_channel.get()
            rospy.loginfo(f"received a mission: '{json}'")
            '''
                each new mission gets a fresh heartbeat status checker
            '''
            heartbeat_message_sender = HeartbeatMessageSender(
                mqtt_client=mqtt_client,
                hover_threshold=20,
                rtl_threshold=60
            )
            heartbeat_message_sender.start_heartbeat()
            reusable_message_senders.add(heartbeat_message_sender)

            try:
                outcome = run_state_machine(json, args)
                rospy.loginfo("FINAL : %s " % outcome)

                heartbeat_message_sender.stop_heartbeat()
            
                if outcome != "mission_completed":
                    is_running = False
            except Exception as e:
                rospy.signal_shutdown(e)
                raise e
        else:
            rospy.logerr("did not receive mission spec")
            is_running = False


    # clean up
    reusable_message_senders.stop()
    if not rospy.is_shutdown():
        rospy.signal_shutdown("Done")
    rospy.spin()



if __name__ == "__main__":
    # TODO create a commandline arg to enable the debugger?
    # import debugpy
    # 5678 is the default attach port in the VS Code debug configurations. Unless a host and port are specified, host defaults to 127.0.0.1
    # debugpy.listen(5678)
    # print("Waiting for debugger attach")
    # debugpy.wait_for_client()
    main()

