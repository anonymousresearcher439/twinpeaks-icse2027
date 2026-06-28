from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states import BaseState


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

        TIMER_NAME = "StatusMessage__send_status_timer"
        self.state.message_senders.add(RepeatTimer(TIMER_NAME, frequency))
        self.state.handlers.add_handler(TIMER_NAME, self._send_status_message)


    def _send_status_message(self, message: dict):
        current_drone_position = self.state.drone.data.location.get_position()
        current_drone_attitude = self.state.drone.data.attitude.get_attitude()
        current_gimbal_attitude = self.state.drone.gimbal.attitude.get_attitude()

        self.state.mqtt_client.publish("update_drone", {
            "uavid": self.state.drone.data.uav_id,
            "status": {
                "status": self.state.drone.data.status.name,
                "mode": self.state.drone.data.mode.name,
                "onboard_pilot": self.state.drone.data.state_name,
                "speed": self.state.drone.data.ground_speed,
                "location": {
                    "latitude": current_drone_position.latitude,
                    "longitude": current_drone_position.longitude,
                    "altitude": current_drone_position.altitude,
                },
                "armed": self.state.drone.data.armed,
                "battery": {
                    "voltage": self.state.drone.data.battery.voltage,
                    "current": self.state.drone.data.battery.current,
                    "level": self.state.drone.data.battery.level,
                },
                "geofence": self.state.drone.data.geofence_status,
                "heartbeat_status" : self.state.drone.data.heartbeat_status.name,
                "gimbal_attitude" : {
                                        "x": current_gimbal_attitude.x,
                                        "y": current_gimbal_attitude.y,
                                        "z": current_gimbal_attitude.z,
                                        "w": current_gimbal_attitude.w
                                    },
                "gimbal_heading" : self.state.drone.gimbal.heading,
                "drone_attitude": {
                                    "x": current_drone_attitude.x,
                                    "y": current_drone_attitude.y,
                                    "z": current_drone_attitude.z,
                                    "w": current_drone_attitude.w
                                },
                "drone_heading": self.state.drone.data.heading
            }
        })