from dataclasses import asdict
import json
from enum import Enum, auto
import random
import time
from typing import (
    Dict,
    List,
    Optional,
    Set,
    Type,
    Union,
)

import rospy

from dr_onboard_autonomy.models.kinematics import LlaPosition
from dr_onboard_autonomy.airlease import (
    AirspaceVolumeName,
    AirLeaserModel,
    AirspaceVolume,
    AirspaceSegment,
    AirLeaseCallback
)
from dr_onboard_autonomy.airlease.messages import (
    Cleanup,
    Land,
    HoverRequest,
)
from dr_onboard_autonomy.airlease.protocol import (
    next_request_number
)

from dr_onboard_autonomy.message_senders import RepeatTimer


class AirLease:
    def __init__(self, state: 'BaseState', cleanup_messages: bool = False):
        self.uav_id = state.drone.uav_name
        self._state = state
        self._protocol_fsm: AirLeaserModel = state.air_lease_protocol_fsm
        self.mqtt = state.mqtt_client

        airlease_status = state.reusable_message_senders.find("airlease_status")
        state.message_senders.add(airlease_status)
        state.handlers.add_handler("airlease_status", self.on_air_lease_status)

        airlease_prefix = type(self).__name__

        protocol_timer_name = f"{airlease_prefix}__protocol_timer"
        protocol_timer = RepeatTimer(protocol_timer_name, 0.1)
        state.message_senders.add(protocol_timer)
        state.handlers.add_handler(protocol_timer_name, self.on_protocol_timer)

        self._cleaned_up_segments = 0

        if cleanup_messages:
            cleanup_timer_name = f"{airlease_prefix}__cleanup_timer"
            airlease_cleanup_timer = RepeatTimer(cleanup_timer_name, 1)
            state.message_senders.add(airlease_cleanup_timer)
            state.handlers.add_handler(cleanup_timer_name, self.send_cleanup_message)
    
    @property
    def airspace(self) -> Optional[AirspaceVolume]:
        return self._protocol_fsm.airspace

    @property
    def lease_id(self) -> Optional[int]:
        return self._protocol_fsm.lease_id
    
    def request_airspace(self, airspace: Union[AirspaceVolume, AirspaceSegment], callback: AirLeaseCallback) -> AirspaceVolumeName:
        """Start requesting the given airspace volume from the ground control station.
        
        The callback will be called every time the air lease is denied and once if/when if granted.

        However, there are cases where the callback **will not** be triggered:
        - If the request itself never reached the ground station (e.g., due to network failure), we do not assume a denial.
        - If the deny response is lost in transit, we will not call the callback (since we never received confirmation of the denial).
        - If the request was rejected due to an out-of-sync state (meaning the airspace itself was not actually assessed), we do not call the callback.

        In short, the callback is only called when:
        - The request reaches the ground station successfully.
        - The request is in sync and properly assessed.
        - A denial response is sent and received.
        - Or if the airspace is granted.

        Note: The callback may be triggered multiple times if the lease is denied multiple times before being granted.
        """
        if not isinstance(airspace, list):
            airspace = [airspace]
        return self._protocol_fsm.update_airspace(airspace, callback)
    
    def on_air_lease_status(self, msg):
        mqtt_msg = msg["data"]
        self._protocol_fsm.receive_response(mqtt_msg)

    def on_protocol_timer(self, msg):
        delta_t = msg["data"]
        self._protocol_fsm.tick(delta_t)
    
    def send_land_message(self):
        """Send a land message to the air lease service
        """
        request_number = next_request_number()
        msg = Land(self.uav_id, request_number)
        self.mqtt.publish(topic="airlease/land", data=asdict(msg), qos=1) # Yes QoS 1 we want to make sure it gets there at least once

    def send_cleanup_message(self, msg):
        """Send a cleanup message to air lease service
        """
        # TODO remove Cleanup messages or maybe make them ordinary requests that
        # require a response
        if "position" not in self._state.message_data:
            rospy.logdebug("on_cleanup_timer - no position data available")
            return

        position = self._state.drone.data.location.get_position() # wgs84 ellipsoid already
        pos_tup = (position.latitude, position.longitude, position.altitude)
        msg = Cleanup(self.uav_id, pos_tup)
        self.mqtt.publish(topic="airlease/cleanup", data=asdict(msg))
    
    def send_hover_message(self, position: Optional[LlaPosition] = None):
        """Send a hover message to the air lease service
        """
        # TODO remove HoverRequests or at least make them ordinary requests that
        # require a response. We're keeping them to make field testing easier
        # (at least for now)
        if not self.airspace:
            rospy.logwarn("send_hover_message - cannot send hover message because we don't have an air lease")
            return
        if position is None:
            try:
                position = self.airspace[-1].end.to_wgs84_ellipsoid()
            except:
                position = self._state.drone.data.location.get_position()
        
        # the request number is a little tricky here This isn't the lease ID,
        # but the request number associated with the specific segment of
        # airspace at the end AirspaceVolume. So we need to calculate the
        # request number based on the current lease ID and the number of
        # segments in the airspace.
        offset = len(self.airspace) - 1
        request_number = self._protocol_fsm.lease_id + offset
        pos_tup = (position.latitude, position.longitude, position.altitude)
        msg = HoverRequest(self.uav_id, request_number, pos_tup)
        
        self.mqtt.publish(topic="airlease/hover", data=asdict(msg))

from dr_onboard_autonomy.states import BaseState
