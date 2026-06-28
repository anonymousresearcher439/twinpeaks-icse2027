import rospy

from .BaseState import BaseState


class Takeoff(BaseState):
    def __init__(self, takeoff_altitude=2.5, altitude_threshold=1.0, **kwargs):
        kwargs["outcomes"] = [
            "succeeded_takeoff",
            "failed_takeoff",
            "error",
            "human_control",
        ]
        super().__init__(**kwargs)
        msg_sender_names = ["extended_state", "relative_altitude"]
        for name in msg_sender_names:
            message_sender = self.reusable_message_senders.find(name)
            self.message_senders.add(message_sender)

        for message_type in msg_sender_names:
            self.handlers.add_handler(message_type, self.on_ros_message)

        # TODO ask about this value
        # The drone is considered close enough to takeoff altitude when its
        # relative altitude is within this many meters
        self.altitude_threshold = altitude_threshold
        self.takeoff_altitude = takeoff_altitude

    def on_entry(self, userdata):
        is_mode_change_success = self.drone.takeoff(self.takeoff_altitude)
        rospy.loginfo(f"TAKEOFF mode activated: {is_mode_change_success}")
        if not is_mode_change_success:
            return "error"

    def on_ros_message(self, message):
        is_data_available = all(
            [
                "extended_state" in self.message_data,
                "relative_altitude" in self.message_data,
            ]
        )
        if not is_data_available:
            return

        min_alt = self.takeoff_altitude - self.altitude_threshold
        rel_alt = self.message_data["relative_altitude"].data

        is_done = all(
            [
                self.message_data["extended_state"].landed_state == 2,
                rel_alt >= min_alt,
            ]  # 2 == LANDED_STATE_IN_AIR
        )
        if is_done:
            return "succeeded_takeoff"
