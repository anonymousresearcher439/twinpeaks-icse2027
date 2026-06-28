import json
from enum import Enum, auto
import random
from typing import (
    Dict,
    List,
    Optional,
    Set,
    Type,
    Union,
)
from dr_onboard_autonomy.state_factory import STANDARD_TRANSITIONS_FLYING
import rospy

from droneresponse_mathtools import Lla

from dr_onboard_autonomy.air_lease_requests import MultiRequest as MultiTunnelRequest
from dr_onboard_autonomy.air_lease_requests import Request as TunnelRequest
from dr_onboard_autonomy.air_lease import AirTunnel, TunnelFunc
from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.message_senders import RepeatTimer
from dr_onboard_autonomy.states.BaseState import BaseState
from dr_onboard_autonomy.states.components.trajectory import HoldingTrajectory

# The AirLeaser state can be initialized in two modes:
# 1. FLYING_MODE - for when we're flying
# 2. GROUND_MODE - for when we're on the ground (before takeoff)
# 
# In the FLYING_MODE, we want to:
# - use ReadMessagesAirborne to get our initial data
# - Use HoldingTrajectory while we run the air lease protocol
# - include the flying outcomes: "succeeded", "error", "failsafe", "abort", "rtl"
#
# In GROUND_MODE, we want to:
# - use ReadMessages to get our initial data
# - set trajectory_class to None
# - include only these necessary outcomes: "succeeded", "error"
# 
# in both modes we want to:
# 1. get the current position (from ReadMessages or ReadMessagesAirborne)
# 2. run our Protocol State Machine
# 3. Regardless of what's going on with our protocol state machine, if we get
#    permission to fly, return "succeeded"

# The Protocol State Machine will have three states:
class _State(Enum):
    REPEATED_REQUESTS = auto() # this state just repeatedly sends requests
    LEASE_DENIED = auto() # this is the state we enter when our air lease is denied
    DISCONNECTED = auto() # this is the state we enter when we get disconnected from the MQTT server

# Each state will have a `run()` method that returns the "name" of the next state.
#
# We start off in the REPEATED_REQUESTS state.
# In this state we ask every X seconds for our air lease.
# if we get disconnected from MQTT, enter the DISCONNECTED state
# if we get denied, enter the LEASE_DENIED state
# if we don't get a response, we remain in the REPEATED_REQUESTS state
# 
# In the LEASE_DENIED state we will:
# - Wait for a certain amount of time and enter the REPEATED_REQUESTS state
#
# In the DISCONNECTED state we will:
# - check if we have a connection
#   - if we do, enter the REPEATED_REQUESTS state
#   - if we don't, remain in the DISCONNECTED state
#
# our states have one method:
#   run() - this is called by AirLeaser._on_timer()
# the run method returns a _State Enum value. This value indicates the next state to enter
# if the return value is the same as the current state, then we remain in the current state.
# the run method will have one argument: delta_t. This is the amount of time that has passed
# since the last time the run method was called (in seconds).

class _RepeatRequestsState:
    def __init__(self, air_leaser: 'AirLeaser', retry_period: float=1.0):
        """This state is active when we are repeatedly sending air lease requests.

        Args:
            air_leaser (AirLeaser): the AirLeaser object that this state belongs to
            retry_period (float): the amount of time to wait between requests
        """
        self.retry_period = retry_period
        self.time_for_next_request = 0.0 # this is the time when we should send our next request in seconds
        self.total_elapsed_time = 0.0 # how much time has elapsed since we entered this state in seconds. It's updated every time run() is called. Specifically the delta_t argument is accumulated here. 
        self.air_leaser = air_leaser
        self.type = _State.REPEATED_REQUESTS
    
    def run(self, delta_t: float):
        time_now = self.update_elapsed_time(delta_t)
        if time_now >= self.time_for_next_request:
            if not self.air_leaser.mqtt_client.is_connected():
                """
                We check the MQTT connection status only when we're about to send a request. This prevents us from spamming the air lease service with extra requests. Normally, we send a request when entering this state. So if we checked the MQTT connection outside of this condition, we could potentially send requests every time the MQTT connection is reestablished.
                """
                return _State.DISCONNECTED

            self.send_request()

        return _State.REPEATED_REQUESTS

    def send_request(self):
        self.air_leaser.send_request()
        self.time_for_next_request = self.total_elapsed_time + random.uniform(self.retry_period/2, self.retry_period*1.5)
    
    def update_elapsed_time(self, delta_t: float):
        """Update the total elapsed time and return it
        """
        self.total_elapsed_time = self.total_elapsed_time + delta_t
        return self.total_elapsed_time


class _LeaseDeniedState:
    def __init__(self, air_leaser: 'AirLeaser', wait_time: float=2.0):
        """This state is active when our air lease is denied.

        We remain in this state for a certain amount of time before going back to the
        RepeatRequestsState.

        Args:
            air_leaser (AirLeaser): the AirLeaser object that this state belongs to
            wait_time (float): the amount of time to wait before going back to the
                RepeatRequestsState
        """
        self.wait_time = wait_time
        self.total_elapsed_time = 0.0
        self.air_leaser = air_leaser
        self.type = _State.LEASE_DENIED

    def run(self, delta_t: float):
        time_now = self.update_elapsed_time(delta_t)
        if time_now >= self.wait_time:
            return _State.REPEATED_REQUESTS
        return _State.LEASE_DENIED

    def update_elapsed_time(self, delta_t: float):
        """Update the total elapsed time and return it
        delta_t is in seconds

        the return value is the number of seconds that have elapsed since we entered this state.
        We assume that the total_elapsed_time is 0 when we enter this state.
        We also assume the delta_t is the amount of time that has passed since the last time run() was called.
        """
        self.total_elapsed_time = self.total_elapsed_time + delta_t
        return self.total_elapsed_time


class _DisconnectedState:
    def __init__(self, air_leaser: 'AirLeaser', _: float=1.0):
        """This state is active when the MQTT client is disconnected.
        
        We remain in this state until the MQTT connection is reestablished. 

        In this state, the system periodically checks the MQTT client's internal
        variable to determine if the connection has been reestablished. Note that
        this is a check of the client object's internal state.

        Args:
            air_leaser (AirLeaser): The AirLeaser object associated with this state.
            _ (float): This argument is not used in this state but is included
                to maintain consistency with the method signatures of other states.
        """
        self.start_time = 0.0
        self.air_leaser = air_leaser
        self.type = _State.DISCONNECTED
    
    def run(self, _: float):
        if self.air_leaser.mqtt_client.is_connected():
            return _State.REPEATED_REQUESTS
        return _State.DISCONNECTED


class AirLeaser(BaseState):
    """This state asks for an air lease until one is granted

    Args:
        tunnel_func Optional[TunnelFunc]: Generates air lease Request args - provided from closures in 
            dr_onboard_autonomy.air_lease
        
        is_flying bool: are we flying? This controls how AirLeaser initializes its internal state 
            if `is_flying` is True, then we will hover in place while we send requests and
            wait for permission to fly. Furthermore we will include certain outcomes by default
            such as the "failsafe" outcome.

            But if `is_flying` is False, then we will not send any setpoints to the flight controler
            and we will only include the "succeed" and "error" outcomes.

            Most of the time this should be True. Set this to false for Takeoff. 

        unwanted_outcomes Set[str]: this is a set of strings. These are outcomes to exclude from the outcome
            argument passed to smach.State. For example, the Takeoff state doesn't want the
            "failsafe" outcome to be possible before lift-off. By default this is the empty
            set. Most of the time you can use the default.
    """
    def __init__(
        self,
        tunnel_func: Optional[TunnelFunc]=None,
        is_flying: bool=True,
        unwanted_outcomes: Set[str]=set(),
        **kwargs
    ):  
        if is_flying:
            # We're in FLYING_MODE
            required_outcomes = ["succeeded", *STANDARD_TRANSITIONS_FLYING.keys()]
            TrajectoryClass = HoldingTrajectory
            ReadMessagesClass: 'Type[ReadMessages]' = ReadMessagesAirborne
        else:
            # We're in GROUND_MODE
            required_outcomes = [
                "error",
                "succeeded",
            ]
            TrajectoryClass = None
            ReadMessagesClass: 'Type[ReadMessages]' = ReadMessages
        
        outcome_set = set(kwargs.get("outcomes", []))
        for other_outcome in required_outcomes:
            outcome_set.add(other_outcome)
        outcome_set = outcome_set - unwanted_outcomes
        self.outcome_set = outcome_set

        kwargs["outcomes"] = list(outcome_set)
        kwargs["trajectory_class"] = TrajectoryClass
        super().__init__(**kwargs)

        self.ReadMessagesClass: 'Type[ReadMessages]' = ReadMessagesClass
        self.unwanted_outcomes = unwanted_outcomes
        self.tunnel_func: TunnelFunc = tunnel_func
        self.mission_builder: 'MissionBuilder' = kwargs["mission_builder"]
        self.current_position: Optional[BriarLla] = None

        self.message_senders.add(self.reusable_message_senders.find("airlease_status"))
        self.handlers.add_handler("airlease_status", self._on_airlease_message)

        self.message_senders.add(RepeatTimer("airlease_timer", 0.25))
        self.handlers.add_handler("airlease_timer", self._on_timer)

        self._current_state = _RepeatRequestsState(self)
        self._request: Optional[Union[TunnelRequest, MultiTunnelRequest]] = None

        self._exit_on_deadlock = "deadlock" in kwargs.get("outcomes", [])
        self.kwargs = kwargs
    
    def on_entry(self, userdata):
        # Read our current position
        pos_reader_kwargs = self.mission_builder.build_kwargs({
            "message_names": ["position"],
            "name": self.name,
            "outcomes": list(self.outcome_set),
            "unwanted_outcomes": self.unwanted_outcomes,
        })
        if self.task_id is not None:
            pos_reader_kwargs["task_id"] = self.task_id
        reader_state = self.ReadMessagesClass(**pos_reader_kwargs)
        outcome = self.execute_substate(reader_state, userdata)
        if outcome != "succeeded":
            return outcome
        output = reader_state.get()
        self.current_position = BriarLla.from_ros(output["position"])

        if self.trajectory:
            self.trajectory.start()
        self._on_timer({"data": 0.0})
    
    def communicate_route_complete(self, current_position: Optional[Lla]=None):
        """Completes last air lease

        Doesn't use AirLeaser._current_position as it is likely to be stale by the time
        this function is called
        """
        if current_position is not None:
            self.air_lease_service.send_done(
                position=[
                    current_position.lat,
                    current_position.lon,
                    current_position.alt
                ]
            )
        else:
            self.air_lease_service.send_done()

    def land(self):
        """Air leasing service does not provide a reply for land requests, so request and move on
        """
        self.air_lease_service.land()
        return "succeeded"
    
    def _on_timer(self, msg):
        delta_t = msg["data"]
        outcome = self._current_state.run(delta_t)
        if outcome == self._current_state.type:
            return
        
        if outcome == _State.REPEATED_REQUESTS:
            self._current_state = _RepeatRequestsState(self)
        elif outcome == _State.LEASE_DENIED:
            self._current_state = _LeaseDeniedState(self)
        elif outcome == _State.DISCONNECTED:
            self._current_state = _DisconnectedState(self)
        else:
            rospy.logerr(f"AirLeaser - internal error unknown next state: {outcome}")

    def _on_airlease_message(self, message: Dict):
        data_payload = json.loads(message["data"].payload)

        is_correct_drone = data_payload["drone_id"] == self.drone.uav_name
        is_approved = data_payload["approved"]
        is_correct_request = data_payload["request_number"] == self._get_request_number()

        is_granted = is_correct_drone and is_correct_request and is_approved
        
        if is_granted:
            rospy.loginfo("AirLeaser - Air lease approved")
            return "succeeded"
        
        if is_correct_drone and is_correct_request:
            # check for deadlock
            if data_payload['deadlock']:
                rospy.logwarn("AirLeaser - Deadlock detected")
                if self._exit_on_deadlock:
                    return "deadlock"
                
            rospy.loginfo("AirLeaser - Air lease denied")
            # enter the LEASE_DENIED state if we're not already in it
            if self._current_state.type != _State.LEASE_DENIED:
                """
                We need to guard against Reinitializing the LEASE_DENIED state every time so that we don't reset our timer every time we get a message from the air lease service.
                """
                self._current_state = _LeaseDeniedState(self)
    
    def _get_request_number(self) -> int:
        """Get the request number from the request we have stored in this object
        """
        if self._request is None:
            rospy.logwarn("AirLeaser - _get_request_number called before a request was created")
            rospy.logwarn("AirLeaser - we must be processing older messages. returning -1")
            return -1 # this is an impossible request number so it should never match a response
        
        if isinstance(self._request, TunnelRequest):
            return self._request.request_number
        
        if isinstance(self._request, MultiTunnelRequest):
            # multi-tunnel requests have a list of requests, each with their own request id
            # but the air leasing service will reference the first request id when it responds
            return self._request.requests[0].request_number
    
    def send_request(self):
        """Sends an air lease request to the air lease service.

        The first time, this will use the tunnel_func to generate the request message.
        Subsequent calls will use the same request message.
        """
        if not self._request:
            # we've not yet initialized our request message, this must be the first time
            # this method has been called
            self._request = self._build_air_lease_request(self.current_position)
        else:
            self._request = self.air_lease_service.generate_final_request(self._request) 
        
        self.air_lease_service.send_request(self._request)
        
        if isinstance(self._request, MultiTunnelRequest):
            rospy.loginfo("AirLeaser - Sent multi request to air lease service")
        else:
            rospy.loginfo("AirLeaser - Sent request to air lease service")
    
    def _build_air_lease_request(self, current_pos: BriarLla) -> Union[TunnelRequest, MultiTunnelRequest]:
        """Internal method: Build a request using our current position and the tunnel_func.
        This will create a request that's ready to send to the air lease service.
        It will fill out everything, including the drone_id and request_number fields.

        If the tunnel_func returns a single AirTunnel, then this will return a TunnelRequest
        If the tunnel_func returns a list of AirTunnels, then this will return a MultiTunnelRequest
        """

        # We start building a request with a temporary value called `pending_request`.
        # However, this method cannot add all the details to this request directly.
        # The `drone_id` and `request_number` fields are populated by calling the
        # `air_lease_service.generate_final_request()` method. This method creates
        # a new request by Copying everything over from our `pending_request` except
        # it overwrites the uninitialized fields (`drone_id` and `request_number`),
        # making it the final request.
        pending_request: Union[TunnelRequest, MultiTunnelRequest] = None

        pos = current_pos.ellipsoid.lla
        tunnel_response = self.tunnel_func(pos)

        if not AirLeaser.check_tunnel_func_result(tunnel_response):
            # raise a value error something bad has happened
            rospy.logerr(f"AirLeaser - tunnel_func returned the wrong type. Actual: '{type(tunnel_response)}' Expected: AirTunnel, Tuple[Lla, Lla, float] or a list of either")
            raise ValueError(f"tunnel_func returned the wrong type. Actual: '{type(tunnel_response)}' Expected: AirTunnel, Tuple[Lla, Lla, float] or a list of either")

        # multi-tunnel functions return a list of AirTunnel objects
        if isinstance(tunnel_response, List):
            rospy.loginfo("AirLeaser - Creating multi tunnel air lease request")
            requests = []
            for air_tunnel in tunnel_response:
                tun_req = self._build_one_tunnel_request(air_tunnel)
                requests.append(tun_req)
            
            pending_request = MultiTunnelRequest(requests=requests)
        # single-tunnel functions return a single AirTunnel object or a Tuple[Lla, Lla, float]
        else:
            rospy.loginfo("AirLeaser - Creating singular tunnel air lease request")
            pending_request = self._build_one_tunnel_request(tunnel_response)

        # last build the final request with the drone_id and request_number fields populated
        return self.air_lease_service.generate_final_request(pending_request)

    @staticmethod
    def check_tunnel_func_result(tunnel_response) -> bool:
        """make sure the tunnel_func returns an acceptable type
        It should return one of the following:

        For multi-tunnel functions:
        - List[AirTunnel]
        - List[Tuple[Lla, Lla, float]]
        - List[Union[AirTunnel, Tuple[Lla, Lla, float]]

        For single-tunnel functions:
        - AirTunnel
        - Tuple[Lla, Lla, float]

        This returns True when the tunnel_func returns a type that matches one of
        the above. This returns False otherwise.
        """
        # for multi-tunnel functions
        if isinstance(tunnel_response, List):
            # go through each to make sure it's either an AirTunnel or a Tuple
            for tunnel in tunnel_response:
                # make sure it's not a list and then call this function again to
                # check the type of each element. We need to make sure it's not
                # a list because we want to avoid a stack overflow
                """
                Ensure the input is not a list before recursively calling this function. This is to prevent a potential stack overflow if the tunnel_func returned a list that referenced itself. This is unlikely, but it's better to be safe.
                """
                if isinstance(tunnel, List):
                    return False
                if not AirLeaser.check_tunnel_func_result(tunnel):
                    return False
            return True
        
        # for single-tunnel functions
        # do we have an AirTunnel?
        if isinstance(tunnel_response, AirTunnel):
            return True
        
        # do we have a tuple?
        if isinstance(tunnel_response, tuple) and len(tunnel_response) == 3:
            start, end, radius = tunnel_response
            is_correct = isinstance(start, Lla) and isinstance(end, Lla) and isinstance(radius, float)

            if is_correct:
                rospy.logwarn("AirLeaser - tunnel_func returned a tuple, but we recommend using the AirTunnel type instead of a tuple.")
            return is_correct
        
        return False

    def _build_one_tunnel_request(self, air_tunnel: AirTunnel):
        """Builds a TunnelRequest from an AirTunnel object
        For internal use only. If you want to build a request, use _build_air_lease_request
        """
        start_pos, end_pos, radius = air_tunnel
        return TunnelRequest(
            drone_id=self.drone.uav_name,
            start_position=[start_pos.lat, start_pos.lon, start_pos.alt],
            end_position=[end_pos.lat, end_pos.lon, end_pos.alt],
            radius=radius,
            request_number=0 # can be anything - this is filled in by the send_request method
        )


from .ReadDroneSensors import ReadMessagesAirborne, ReadMessages
from dr_onboard_autonomy.mission_helper import MissionBuilder