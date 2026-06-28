import rospy
from droneresponse_mathtools import Lla
from queue import Queue
from sensor_msgs.msg import NavSatFix
from typing import Optional

from dr_onboard_autonomy.logutils import DebounceLogger
from dr_onboard_autonomy.air_lease import make_waypoint_multi_air_tunnel_func
from dr_onboard_autonomy.briar_helpers import BriarLla, convert_Lla_to_LlaDict, convert_tuple_to_LlaDict
from dr_onboard_autonomy.states import AirLeaser, Offboard, ReadPosition
from dr_onboard_autonomy.states.components.trajectory import WaypointTrajectory

from dr_onboard_autonomy.message_senders import RepeatTimer

from .BaseState import BaseState


_TRANSITION_DELAY = 3.0


"""Example JSON:

{
    "name": "Takeoff",
    "args": {
        "altitude": 7,
        "speed": 1.5,
        "altitude_threshold": 1.0

    },
    "transitions": [
        {
            "target": "BriarHover",
            "condition": "succeeded_takeoff"
        },
        {
            "target": "Land",
            "condition": "failed_takeoff"
        }
    ]
}

"""

class Takeoff(BaseState):
    """
    Args:
        altitude: height in meters above current altitude
    """
    def __init__(self, altitude:float=None, speed:float=1.5, altitude_threshold=1.0, **kwargs):
        kwargs["outcomes"] = [
            "succeeded_takeoff",
            "failed_takeoff",
            "error",
            "human_control",
            "rtl"
        ]
        super().__init__(heartbeat_handler=False, trajectory_class=WaypointTrajectory, **kwargs)
        self.message_senders.add(self.reusable_message_senders.find("relative_altitude"))
        self.handlers.add_handler("relative_altitude", self.store_rel_alt)

        self.message_senders.add(self.reusable_message_senders.find("position"))
        self.handlers.add_handler("position", self.store_position)
        self.handlers.add_handler("position", self.on_position_message)

        self.message_senders.add(RepeatTimer("print_alt", 0.333))
        self.handlers.add_handler("print_alt", self.log_rel_alt)

        self.altitude_threshold = altitude_threshold
        self.takeoff_altitude = altitude
        self.speed = speed

        if 'name' not in kwargs:
            kwargs['name'] = self.name
        self._pass_through_kwargs = kwargs
        self.offboard_switch = Offboard(**kwargs)

        self._air_leaser: Optional[AirLeaser] = None
        self._pos_return_channel = Queue()
        self._position_reader = ReadPosition(return_channel=self._pos_return_channel, **kwargs)

        self.rel_alt = None
        self.position = None
        self._start_position = None
        self._final_position = None

        self.debounce_logger = DebounceLogger()


    def on_entry(self, userdata):
        if self.takeoff_altitude is None:
            self.takeoff_altitude = self.drone.get_param_real("MIS_TAKEOFF_ALT")

        outcome = self._position_reader.execute(userdata=userdata)
        rospy.loginfo(f"Takeoff - initial position retrieval outcome: {outcome}")
        if outcome != "succeeded":
            return outcome
        current_pos: BriarLla = self._pos_return_channel.get()
        final_pos: Lla = current_pos.ellipsoid.lla.move_ned(0, 0, -1.0 * self.takeoff_altitude)
        waypoint_air_tunnel_func = make_waypoint_multi_air_tunnel_func(end_pos=final_pos)
        self._air_leaser = AirLeaser(
            tunnel_func=waypoint_air_tunnel_func,
            hold_position=False,
            unwanted_outcomes={'human_control', 'rtl'},
            **self._pass_through_kwargs
        )
        outcome = self._air_leaser.execute(userdata=userdata)
        if outcome != "succeeded":
            return outcome
        rospy.loginfo("Takeoff - air lease succeeded")

        '''
        don't switch to offboard mode until air lease granted - 
        don't want requirement to provide set points
        '''
        outcome = self.offboard_switch.execute(userdata)
        rospy.loginfo(f"offboard mode change outcome: {outcome}")
        if outcome != "succeeded":
            return outcome

        rospy.loginfo(f"Taking off to {self.takeoff_altitude} meters")
        self.trajectory.start()
    
    def store_rel_alt(self, message):
        self.rel_alt = message['data'].data
    
    def store_position(self, message):
        pos: NavSatFix  = message['data']
        pos_tup = pos.latitude, pos.longitude, pos.altitude
        self.position = BriarLla(convert_tuple_to_LlaDict(pos_tup), is_amsl=False)
        if self._start_position is None:
            self._start_position = self.position
            final_pos = self._start_position.ellipsoid.lla.move_ned(0, 0, -1.0 * self.takeoff_altitude)
            final_pos = convert_Lla_to_LlaDict(final_pos)
            self._final_position = BriarLla(final_pos, is_amsl=False)
            self.trajectory.fly_to_waypoint(self._final_position.amsl.lla, self.speed)
        

    def log_rel_alt(self, _):
        if self.rel_alt is not None:
            self.debounce_logger.info(f"Takeoff state: relative altitude: {round(self.rel_alt, 3)} meters. Takeoff ALT = {self.takeoff_altitude} meters.", 1.0, 1)
        if self.position is not None:
            alt_amsl = self.position.amsl.lla.altitude
            self.debounce_logger.info(f"Takeoff State: altitude AMSL: {round(alt_amsl, 3)}", 1.0, 2)

    def on_position_message(self, message):
        pos = message["data"]
        current_pos = Lla(pos.latitude, pos.longitude, pos.altitude)
        
        distance = current_pos.distance(self._final_position.ellipsoid.lla)

        self.debounce_logger.info(f"Takeoff State: current position: {current_pos}", 1.0, 3)
        self.debounce_logger.info(f"Takeoff State: distance to target: {round(distance, 3)} meters", 1.0, 4)
        self.debounce_logger.info(f"Takeoff State: trajectory.is_done() == {self.trajectory.is_done()}", 1.0, 5)

        if distance < self.altitude_threshold and self.trajectory.is_done():
            self._air_leaser.communicate_route_complete(current_position=current_pos)
            return "succeeded_takeoff"


