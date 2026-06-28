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

    # NOTE: Forcing a cast on the typing due to strange typing on `rospy.get_param`
    cli_overrides = T.cast(
        Overrides,
        [
            (("name",), rospy.get_param("~uav_name", "Polkadot")),
            (("mqtt", "host"), rospy.get_param("~mqtt_host", "mqtt")),
            (("performance_analysis",), rospy.get_param("~performance_analysis", False)),
            (("comment",), rospy.get_param("~comment", "")),
        ]
    )

    config = load_model(DRConfig, config_file, cli_overrides)
    rospy.loginfo(config)
    return config


def recv_mission(mission_builder: MissionBuilder):
    # receive a mission
    kwargs = mission_builder.build_kwargs({})
    mission_receiver = ReceiveMission(**kwargs)
    outcome = mission_receiver.execute(None)
    if outcome != "succeeded":
        rospy.logerr("Failed to receive a mission. Exiting.")
        exit(1)
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
        performance_analysis=config.performance_analysis,
        system_id=config.system_id,
    )
    mission_builder.setup()
    # done initalizing
    # note OnboardPilot only becomes read when mission_builder.setup() returns
    rospy.loginfo("OnboardPilot is ready!")

    while True:
        # now we can get a mission from the ground station
        mission_json = recv_mission(mission_builder)
        # build the mission
        mission = mission_builder.build_mission(mission_json)
        # execute the mission
        outcome = mission.execute(None)
        rospy.loginfo(f"Mission outcome: {outcome}")
        if outcome != "mission_completed":
            break

    # clean up
    # TODO make this better
    mission_builder.mandatory_args['reusable_message_senders'].stop()
    if not rospy.is_shutdown():
        rospy.signal_shutdown("Done")
    rospy.spin()



if __name__ == "__main__":
    main()
