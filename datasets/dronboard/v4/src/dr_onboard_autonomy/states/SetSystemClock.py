import json
from queue import Queue
import time
from datetime import datetime, timezone

import rospy
from .BaseState import BaseState

from dr_onboard_autonomy.models import TimeRef, TimeSource, GpsPositionMessage

from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.state_factory import register_state, STANDARD_TRANSITIONS_FLYING


@register_state(default_transitions={"error": "failure", "skipped": "succeeded"})
class SetSystemClock(BaseState):
    def __init__(self, reusable_message_senders=None, **kwargs):
        """Set the system clock using time reference message from the flight controller.
        """
        all_outcomes = set(["succeeded", "skipped", "error"])
        for outcome in kwargs.get("outcomes", []):
            all_outcomes.add(outcome)
        kwargs["outcomes"] = list(all_outcomes)

        super().__init__(reusable_message_senders=reusable_message_senders, **kwargs)
        timer_name = f"{__name__}_timeout" # unique name for the timer
        self.message_senders.add(RepeatTimer(timer_name, 5))
        self.handlers.add_handler(timer_name, self.on_timer)

        self.message_senders.add(reusable_message_senders.find("time_reference"))
        self.handlers.add_handler("time_reference", self.on_time_ref)

        self.message_senders.add(reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.on_position)

        self._is_gps_fix = False
    
    def on_entry(self, userdata):
        rospy.loginfo("Attempting to set the system clock using GPS time...")
    
    def on_timer(self, _):
        rospy.logwarn("Could not update the system time using GPS time. Exiting.")
        return "skipped"

    
    def on_position(self, message):
        position: GpsPositionMessage = message['data']
        self._is_gps_fix = position.is_fix
    
    def on_time_ref(self, message):
        
        time_ref: TimeRef = message['data']
        if time_ref.source == TimeSource.UNKNOWN:
            rospy.logwarn(f"{__name__} Not setting the system time because the Time source is unknown.")
            return
        if not self._is_gps_fix:
            rospy.logwarn(f"{__name__} Not setting the system time because a GPS fix is not available.")
            return
        return self._set_time(time_ref)
    
    def _set_time(self, time_ref: TimeRef):
        try:
            time.clock_settime_ns(time.CLOCK_REALTIME, time_ref.adjusted_timestamp_ns())
        except PermissionError as e:
            rospy.logwarn(f"Could not set the system time: {str(e)}")
            return "skipped"
        
        # log the time
        utc_datetime = datetime.fromtimestamp(time_ref.adjusted_timestamp_ns() / 1_000_000_000, tz=timezone.utc)
        local_datetime = utc_datetime.astimezone()

        rospy.loginfo(f"System time set to: {time_ref.adjusted_timestamp_ns()}")
        rospy.loginfo(f"    UTC time:   {utc_datetime}")
        rospy.loginfo(f"  Local time:   {local_datetime}")

        return "succeeded"
