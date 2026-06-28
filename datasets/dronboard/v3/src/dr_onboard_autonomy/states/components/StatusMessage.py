from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states import BaseState
from dr_onboard_autonomy.logutils import DebounceLogger

class StatusMessage2:
    """Component to build and send status messages over MQTT
    """
    def __init__(self, state: 'BaseState', frequency: float = 1.0):
        """
        Args:
            state (BaseState instance): The state that this component is a part of
            frequency (float): How often to send the status message
        """
        self.state: 'BaseState' = state
        self.debounce_logger = DebounceLogger()
        TIMER_NAME = "StatusMessage__send_status_timer"
        self.state.message_senders.add(RepeatTimer(TIMER_NAME, frequency))
        self.state.handlers.add_handler(TIMER_NAME, self._send_status_message)
        required_msg_senders = [
            "battery",
            "imu",
            "compass_hdg",
            "velocity",
            "position",
            "relative_altitude",
            "state",
        ]
        for sender in required_msg_senders:
            self.state.message_senders.add(self.state.reusable_message_senders.find(sender))


    def _send_status_message(self, message: dict):
        current_gimbal_attitude = self.state.drone.gimbal.get_attitude()

        output = self.state.drone.data.to_dict()
        output.update({
            "timestamp": self.state.mqtt_client.timestamp(),
        })
        output_status: dict = output["status"]
        output_status.update({
            "gimbal_attitude": None if not current_gimbal_attitude else {
                "x": current_gimbal_attitude.x,
                "y": current_gimbal_attitude.y,
                "z": current_gimbal_attitude.z,
                "w": current_gimbal_attitude.w
            },
            "gimbal_heading" : self.state.drone.gimbal.heading,
        })
        self.debounce_logger.info(f"StatusMessage: sending status message: {output}", 1.5, 999)
        self.state.mqtt_client.publish("update_drone", output, qos=0)
        self.state.local_mqtt_client.publish("update_drone", output, qos=0)
