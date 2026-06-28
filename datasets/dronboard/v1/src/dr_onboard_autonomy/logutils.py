"""
Provides logging utils
Example:
from dr_onboard_autonomy.logutils import DebounceLogger
import time

debouce_logger = DebounceLogger()
for _ in range(10):
    debouce_logger.info("Hello, world", 2)
    time.sleep(0.5)

"""
import time

import rospy


class DebounceLogger:
    """Wraps rospy.loginfo with debounce logic to prevent repetitive logging of
    the same message. The `DebounceLogger` keeps track of the last time each
    message was logged and only logs a message if the specified delay period has
    elapsed since the last time it logged that message. This prevents the log
    output from filling with the same message over and over, which could happen
    if the same message is logged multiple times within a short period of time.

    Example:

    # Create a DebounceLogger instance
    logger = DebounceLogger()

    # Log a message every 5 seconds at most
    while True:
        logger.info("Hello, world!", 5.0)
        time.sleep(1.0)
            
    """
    def __init__(self):
        self.time_table = {}
    
    def info(self, message: str, delay_seconds: float, key=None):
        """
        Logs the specified `message` using `rospy.loginfo` if the delay period
        specified by `delay_seconds` has elapsed since the last time that the
        same `message` (or `key`, if provided) was logged.

        Args:
            message (str): The message to log.
            
            delay_seconds (float): The delay period in seconds. If a message (or
                key, if provided) has been logged within this period, it will not
                be logged again until the period has elapsed.
                
            key (optional, any): Optional key to use to identify the last time
                we logged this kind of message. If not specified, the message
                itself is used as the key. When using a key, you can log
                slightly different messages with the same key. For example, if
                you wanted to log the same message with different values for a
                variable, you could use the format string (or an int) as the
                key. This is useful when you want to log similar messages
                without flooding the logs with identical messages. And this was
                originally created so we can log trajectory data without
                flooding the logs with the same type of message over and over.
        """
        now = time.time()

        if key is None:
            key = message

        last_time = self.time_table.get(key, 0)
        if last_time + delay_seconds < now:
            self.time_table[key] = now
            rospy.loginfo(message)