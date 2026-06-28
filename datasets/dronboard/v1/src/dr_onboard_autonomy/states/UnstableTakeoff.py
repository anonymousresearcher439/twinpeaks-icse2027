import time
import rospy

from dr_onboard_autonomy.message_senders import RepeatTimer

from .BaseState import BaseState


_TRANSITION_DELAY = 3.0


class UnstableTakeoff(BaseState):
    def __init__(self, takeoff_altitude=7, altitude_threshold=1.0, **kwargs):
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
        self.handlers.add_handler("extended_state", self.on_extended_state)
        self._in_air_time = None
        self._count = 0

        # TODO ask about this value
        # The drone is considered close enough to takeoff altitude when its
        # relative altitude is within this many meters
        self.altitude_threshold = altitude_threshold
        self.takeoff_altitude = takeoff_altitude

        if 'name' not in kwargs:
            kwargs['name'] = self.name
        self.takeoff_switcher = _TakeoffModeSwitcher(**kwargs)

    def on_entry(self, userdata):
        self.takeoff_altitude = self.drone.get_param_real("MIS_TAKEOFF_ALT")
        rospy.loginfo(f"Taking off to {self.takeoff_altitude} meters")

        takeoff_mode_outcome = None
        for i in range(5):
            takeoff_mode_outcome = self.takeoff_switcher.execute(userdata)
            rospy.loginfo(f"takeoff mode change outcome: {takeoff_mode_outcome}")
            if takeoff_mode_outcome == "succeeded":
                break
            if takeoff_mode_outcome == "takeoff_error":
                rospy.logwarn("Could not switch to AUTO.TAKEOFF mode")
                continue
            else:
                return takeoff_mode_outcome
        
        if takeoff_mode_outcome != "succeeded":
            return "error"
        
        rospy.loginfo(f"TAKEOFF mode change {takeoff_mode_outcome}")
    
    def on_extended_state(self, message):
        if self._in_air_time is not None:
            return
        
        if message['data'].landed_state == 2:
            self._in_air_time = time.time()

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

        if self._count % 125 == 0:
            rospy.loginfo(f"Takeoff state: relative altitude at {rel_alt} meters. Takeoff ALT = {self.takeoff_altitude} meters")
        self._count = self._count + 1

        if self._in_air_time is None:
            return
        
        time_in_air = time.time() - self._in_air_time

        is_done = all(
            [
                rel_alt >= min_alt,
                time_in_air >= _TRANSITION_DELAY,
                self.message_data["extended_state"].landed_state == 2,
            ]  # 2 == LANDED_STATE_IN_AIR
        )
        if is_done:
            return "succeeded_takeoff"


class _TakeoffModeSwitcher(BaseState):
    """This switches the drone to takeoff mode

    The 'succeeded' outcome will occur when the drone's mode is "AUTO.TAKEOFF"
    The 'error' outcome if the program exits
    The 'takeoff_error' if the mode is not "AUTO.TAKEOFF"
    """

    def __init__(self, timeout=5, data=None, **kwargs):
        kwargs["outcomes"] = ["succeeded", "error", "takeoff_error"]
        super().__init__(**kwargs)

        self.message_senders.add(self.reusable_message_senders.find("state"))
        self.handlers.add_handler("state", self.on_state_message)

        self.message_senders.add(RepeatTimer("takeoff_error", timeout))
        self.handlers.add_handler("takeoff_error", self.on_takeoff_error)

    
    def on_entry(self, userdata):
        rospy.loginfo("switching to AUTO.TAKEOFF mode")
        self.drone.takeoff()
    
    def on_state_message(self, message):
        drone_state = message["data"]
        if drone_state.mode == "AUTO.TAKEOFF":
            return "succeeded"
    
    def on_takeoff_error(self, _):
        return "takeoff_error"