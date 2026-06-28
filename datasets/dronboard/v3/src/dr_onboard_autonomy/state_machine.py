#!/usr/bin/env python3
# pylint is flagging false positives
# pylint: disable=no-member

import os
import typing as T
from pathlib import Path
os.environ['MAVLINK20'] = "1"

import rospy

from dr_onboard_autonomy.mission_helper import MissionBuilder
from dr_onboard_autonomy.states import ReceiveMission
from dr_onboard_autonomy.common.loaders import load_model, Overrides
from dr_onboard_autonomy.models import DRConfig

def load_config():
    config_file = Path(os.environ.get("DR_CONFIG_FILE", "/etc/dr/config.toml"))
    rospy.loginfo(f"Load config: {config_file}")

    # any provided defaults force the use of a particular ros param overriding config file inputs
    cli_ros_params: T.Tuple[T.Tuple[str], str, T.Optional[T.Any]] = (
        (("name",), "~uav_name", None),
        (("mqtt", "host"), "~mqtt_host", None),
        (("mqtt_local", "host"), "~mqtt_local_host", "mqtt_local"), 
        (("performance_analysis",), "~performance_analysis", False),
        (("comment",), "~comment", ""),
        (("gimbal", "type"), "~gimbal_type", None),
        (("fcu", "type"), "~fcu_type", None),
    )

    cli_overrides = []
    for config_map, param, default in cli_ros_params:
        try:
            if default is not None:
                cli_overrides.append((config_map, rospy.get_param(param, default)))
            else:
                cli_overrides.append((config_map, rospy.get_param(param)))
        except KeyError:
            # ignore cli parameters that are both not provided and have no default value
            pass

    # NOTE: Forcing a cast on the typing due to strange typing on `rospy.get_param`
    cli_overrides = T.cast(
        Overrides,
        cli_overrides
    )

    config = load_model(DRConfig, config_file, cli_overrides)
    rospy.loginfo(config)
    return config


def recv_mission(mission_builder: MissionBuilder, previous_outcome=None):
    # receive a mission
    kwargs = mission_builder.build_kwargs({
        "previous_outcome": previous_outcome,
    })
    mission_receiver = ReceiveMission(**kwargs)
    outcome = mission_receiver.execute(None)
    if outcome != "succeeded":
        rospy.logerr("Failed to receive a mission. Exiting.")
        rospy.signal_shutdown("Failed to receive a mission")
        return
    mission_json: str = mission_receiver.return_channel.get()
    return mission_json


def main():
    # the initalization process
    rospy.init_node("OnboardPilot", log_level=rospy.INFO)
    rospy.loginfo("Starting OnboardPilot")
    config = load_config()

    if config.performance_analysis:
        rospy.loginfo("Performance Analysis is enabled")
        import time
        import dr_onboard_autonomy.states.collections.performance_analysis as perf_module
        start_time = time.perf_counter_ns()
        perf_module.setup_benchmarking(start_time, config.comment)
        rospy.on_shutdown(perf_module.stop_benchmarking)
    mission_builder = MissionBuilder(
        uav_id=config.name,
        mqtt_client=config.mqtt.host,
        local_mqtt_client=config.mqtt_local.host,
        performance_analysis=config.performance_analysis,
        system_id=config.system_id,
        drone_config=config
    )
    mission_builder.setup()
    # done initalizing
    # note OnboardPilot only becomes read when mission_builder.setup() returns
    rospy.loginfo("OnboardPilot is ready!")

    outcome = None
    
    while True:
        # now we can get a mission from the ground station
        mission_json = recv_mission(mission_builder, outcome)
        if mission_json is None:
            break
        # in case the drone has been sitting around for a while
        # we need to reset the home position to account for drift
        # See: https://github.com/DroneResponse/DR-OnboardAutonomy/issues/468
        mission_builder.reset_home_position()
        # build the mission
        mission = mission_builder.build_mission(mission_json)
        # execute the mission
        outcome = mission.execute(None)

        rospy.loginfo(f"Mission outcome: {outcome}")
        send_mission_outcome(config.name, outcome, mission_builder.mandatory_args['mqtt_client'])
        send_mission_outcome(config.name, outcome, mission_builder.mandatory_args['local_mqtt_client'])

        if outcome != "mission_completed":
            break

    # clean up
    # TODO make this better
    mission_builder.mandatory_args['reusable_message_senders'].stop()
    if not rospy.is_shutdown():
        rospy.signal_shutdown("Done")
    rospy.spin()
    exit(0)

def send_mission_outcome(uav_id: str, outcome: str, mqtt_client: 'MQTTClient'):
    if mqtt_client.is_connected():
        msg = {
            "timestamp": mqtt_client.timestamp(),
            "uavid": uav_id,
            "outcome": outcome,
        }
        mqtt_client.publish("mission_outcome", msg, qos=1)

def debug_me():
    import debugpy
    # This is for the "Python: Remote Attach" configuration in VS Code.
    # port 5678 is the default that VS Code uses for debugging.
    debugpy.listen(5678)
    print("Waiting for debugger attach")
    debugpy.wait_for_client()

from dr_onboard_autonomy.mqtt_client import MQTTClient

if __name__ == "__main__":
    # TODO create a commandline arg to enable the debugger?
    # debug_me()
    main()
