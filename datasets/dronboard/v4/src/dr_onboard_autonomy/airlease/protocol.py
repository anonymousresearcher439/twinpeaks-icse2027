from dataclasses import asdict, field
import enum 
import json
import random
import time
from typing import (
    Any,
    Callable,
    Dict,
    List,
    NamedTuple,
    Set,
    Tuple,
    Optional,
    TypedDict,
    Union,
    
)

from transitions import Machine, State
from paho.mqtt.client import MQTTMessage

from dr_onboard_autonomy.mqtt_client import MQTTClient

from .messages import (
    Cleanup,
    HoverRequest,
    Land,
    MultiRequest as MultiTunnelRequest,
    Request as SingleTunnelRequest,
)
from .airspace_volume import AirspaceVolume, AirspaceSegment, AirspaceVolumeName


# This is the default callback. It does nothing.
NO_OP = lambda *args, **kwargs: None


class AirLeaseResponse(TypedDict):
    drone_id: str
    request_number: int
    current_lease: int
    approved: bool
    deadlock: bool


class AirLeaseOutcome(NamedTuple):
    approved: bool
    deadlock: bool


AirLeaseCallback = Callable[[AirspaceVolumeName, AirspaceVolume, AirLeaseOutcome], Any]


RequestNumber = int
"""
We want to only send a request _ONCE_. if we don't get a response, we want to retry.
But We will retry with a new request number. This is because the ground control station
will only accept a request number that is greater than the previous request number.
If we reused a request number, the ground control station might reject it because it's
already seen that request number.

Anyway this means we can end up with multiple requests in flight that all refer to the
same AirspaceVolume.
"""


def _only_once(func):
    """Return a function that only calls `func` once.
    You can call the wrapped function as many times as you want, but it will only call `func` once.
    """
    was_called = False
    def wrapper(*args, **kwargs):
        nonlocal was_called
        if was_called:
            return
        was_called = True
        return func(*args, **kwargs)
    return wrapper


class AirspaceRecord(NamedTuple):
    name: AirspaceVolumeName
    airspace: AirspaceVolume
    approved_callback: AirLeaseCallback 
    denied_callback: AirLeaseCallback


class _TimeAccumulator:
    def __init__(self, time_limit: float):
        self._time_limit = time_limit
        self._time = 0.0

    def update(self, delta_time: float):
        self._time += delta_time
    
    def is_time_up(self) -> bool:
        return self._time >= self._time_limit
    
    def time_remaining(self) -> float:
        return self._time_limit - self._time
    
    @property
    def time(self):
        return self._time


_previous_request_number = 0
def next_request_number():
    """Make sure the request numbers are actually unique.
    
    If we use Milliseconds since the epoch, then we might end up with duplicate
    if we create two requests in the same millisecond. So we need to make sure
    that the request number is always unique. 

    This function assume no request will be made with more than 1000 tunnels...
    """
    global _previous_request_number
    result = max(_previous_request_number + 1, time.time_ns())
    _previous_request_number = result
    return result


class States(enum.Enum):
    IDLE = enum.auto()
    SENDING_REQUEST = enum.auto()
    WAITING_RESPONSE = enum.auto()
    RECEVING_RESPONSE = enum.auto()
    LEASE_DENIED = enum.auto()
    DISCONNECTED = enum.auto()


class AirLeaserModel:
    """This is used by the AirLeaser component. This implements the airleaser protocol.

    The import methods:
        - update_airspace
        - receive_response
        - tick

    The import properties:
        - lease_id
        - airspace
    
    The air leaser component will need to wire response messages to the
    receive_response method. It will also need to wire timer messages to the
    tick method.
    
    The state or the component will need call update_airspace when it wants a
    new air lease.

    The component needs to tick the AirLeaserModel repeatedly. This is how the
    state machine keeps track of time. The component should tick the
    AirLeaserModel at a regular interval.

    It's called the AirLeaserModel because it's the "model" used by the
    transitions library's.

    Here's how the transitions library describes the model:

    >The actual stateful structure. It's the entity that gets updated during
    transitions. It may also define actions that will be executed during
    transitions. For instance, right before a transition or when a state is
    entered or exited.
    """
    def __init__(self, uav_id: str, mqtt: MQTTClient):
        self.uav_id = uav_id
        self.mqtt = mqtt

        # The airspace we want. If it's _not_ None and it doesn't match the
        # current airspace, then we have work to do. The protocol state machine
        # is always trying to get to reach a state where the target_airspace is
        # the same as the current_airspace.
        self.target_airspace: Optional[AirspaceRecord] = None

        # All the "in-flight" requests. These are requests that have been sent
        # but we haven't received a response yet. It's possible for any one of
        # these requests to be approved or denied. We're just one message away
        # from knowing the outcome.
        self.pending_requests: Dict[RequestNumber, AirspaceRecord] = {}

        # This is our current air lease (to the best of our knowledge).
        # Hopefully this matches the ground control station's view of our air
        # lease. If it doesn't, we will need to update this field. Every response
        # from the ground control station will tell us what our current air lease is.
        self._current_lease_id: int = 0
        self._current_airspace: Optional[AirspaceRecord] = None

        # This is a counter that we use to assign a unique "name" to each
        # airspace volume. every time update_airspace is called, we create a new
        # name for the provided airspace volume
        def _airspace_counter():
            counter = 1
            while True:
                yield counter
                counter += 1

        self._airspace_counter_gen = _airspace_counter()

        # This is a dictionary that holds state specific data. We use this to
        # hold timers and other state specific objects. We don't want to clutter
        # self with a bunch of fields that are only used in one state.
        self._state_date = {}
    
    def _next_airspace_id(self) -> AirspaceVolumeName:
        return next(self._airspace_counter_gen)

    def update_airspace(self, airspace: AirspaceVolume, callback: AirLeaseCallback=NO_OP) -> AirspaceVolumeName:
        """This is the main method that the air leasing component will use to request airspace volumes.

        Args:
            airspace: The airspace volume we want to request.
            callback: This is a function that will be called when we find out if the request was approved or denied.
        """
        # NOTE: Yes, the method is actually needed even though it's just a
        # wrapper around the trigger. the trigger itself doesn't always return
        # the name of the airspace volume. sometimes it returns True. so we need
        # this method to always return the name of the airspace volume. Also
        # when we call `update_target_airspace_volume` we trigger the state
        # machine and we end up calling on_update_target_airspace_volume. This
        # is the method that does all the work. 

        self.update_target_airspace_volume(airspace, callback) # this is the trigger
        return self.target_airspace.name 

    def on_update_target_airspace_volume(self, airspace: AirspaceVolume = None, callback=NO_OP) -> AirspaceVolumeName:
        """We run this before we transition to the SENDING_REQUEST state. 
        We validate the request our internal state.

        If the arguments are ok, then we can allow the transition to happen.
        When we transition to the SENDING_REQUEST state, we will send the request to the ground control station.
        But this is where we collect the information we need to send the request.

        This trigger can be called from any of the stable states.

        About the callback. We make a best effort to call the callback when we find out
        if the request was approved or denied. We can't guarantee that the callback will
        be called. If a response message is lost, then we may never findout.

        
        So we always call the callback when we realize the airspace was approved. it doesn't matter
        if the target airspace doesn't match the approved airspace. We still call the callback.

        For example, suppose you ask for airspace X, and before hearing back, you ask for airspace Y.
        Then airspace X is approved. We will call the callback for X. Even though we're not trying to
        get X anymore. We still call the callback.

        Things are a little different in case the airspace is denied. We only call the callback if

        The response is about the target airspace. So suppose again you ask for airspace X, then Y.
        If we get a deny message for X, we will _NOT_ call the callback for X. Because we're not trying
        to get X anymore.

        However, if we get a response message for Y, then we will call Y's callback. 
        """
        if len(airspace) < 1:
            # TODO should we send the land request instead?
            # Like maybe if you send the request with no airspace, we should take that to mean you want to end your lease and not replace it with a new one.
            raise ValueError(f"Expected at least one airspace segment. Got {airspace}. We cannot request an airspace volume with no segments.")
        
        self.target_airspace = AirspaceRecord(
            name=self._next_airspace_id(),
            airspace=airspace,
            approved_callback=_only_once(callback),
            # we need to wrap this in only_once because we only want to call the
            # callback once. there are a few different ways we might find out
            # that our request was approved.
            # 1. we get a response message that directly says it was approved
            # 2. the response message approving the air lease gets lost and we
            # find out from a follow up deny message. All response messages tell
            # us our current lease ID. So we might get a response that tells us
            # our current lease maps back to this air space even though we never
            # received the original response message informing us of the
            # approval. It's easier to do the house keeping if we protect the
            # callback from being called more than once right here.
            
            denied_callback=callback, # we allow this to be called multiple times. 
            # we need to know if the request was denied because of a deadlock
            # or if it's just denied. We will make a best effort to call this
            # but there are plenty reasons why we might not be able to call it.
        )

        return self.target_airspace.name
    
    @property
    def lease_id(self) -> int:
        return self._current_lease_id

    @property
    def airspace(self) -> Optional[AirspaceVolume]:
        """Our current air space
        """
        if self._current_airspace:
            return self._current_airspace.airspace
        return None
    
    @property
    def state_data(self) -> Any:
        return self._state_date[self.state]
    
    @state_data.setter
    def state_data(self, value: Any) -> None:
        self._state_date[self.state] = value
    
    def on_enter_idle(self, *args, **kwargs) -> None:
        pass

    def on_exit_idle(self, *args, **kwargs) -> None:
        pass

    def on_enter_sending_request(self, *args, **kwargs) -> None:
        """This is where we create a request object and send it to the ground control station (if we're connected).
        Then we add the in-flight request to the pending requests.
        lastly we transition to the WAITING_RESPONSE state.

        If we're not connected, we transition to the DISCONNECTED state.
        """
        request_number = next_request_number()
        # we need to create the request object
        # we either have 1 airspace segment or multiple
        if len(self.target_airspace.airspace) == 1:
            start = self.target_airspace.airspace[0].start.to_wgs84_ellipsoid()
            end = self.target_airspace.airspace[0].end.to_wgs84_ellipsoid()

            start_tup = (start.latitude, start.longitude, start.altitude)
            end_tup = (end.latitude, end.longitude, end.altitude)
            radius = self.target_airspace.airspace[0].radius
            request = SingleTunnelRequest(
                request_number=request_number,
                current_lease=self.lease_id,
                drone_id=self.uav_id,
                start_position=start_tup,
                end_position=end_tup,
                radius=radius,
            )
        else:
            tunnels = []
            for i, segment in enumerate(self.target_airspace.airspace):
                start = segment.start.to_wgs84_ellipsoid()
                end = segment.end.to_wgs84_ellipsoid()
                start_tup = (start.latitude, start.longitude, start.altitude)
                end_tup = (end.latitude, end.longitude, end.altitude)
                radius = segment.radius
                request = SingleTunnelRequest(
                    request_number=request_number + i,
                    current_lease=self.lease_id,
                    drone_id=self.uav_id,
                    start_position=start_tup,
                    end_position=end_tup,
                    radius=radius,
                )
                tunnels.append(request)
            request = MultiTunnelRequest(
                requests=tunnels,
            )
        if not self.mqtt.is_connected():
            self.lost_connection()
            return
        payload = asdict(request)
        self._send_request(payload)
        self.pending_requests[request_number] = self.target_airspace
        
        self.request_sent() # This is a trigger that moves us to WAITING_RESPONSE state
    
    def _send_request(self, payload: Dict[str, Any]) -> None:
        self.mqtt.publish(topic="airlease/request", data=payload, qos=0)
    
    def _update_current_lease(self, lease_id: RequestNumber) -> None:
        """We call this method when the air leaser tells us our current lease is
        different from what we think it is.

        So we're going to update our current air lease to match as best we can.

        If the request number is in the pending requests, we can reference the
        request itself. In this case we have information about the bounds of our
        air lease.

        If the request number refers to a request that we have never seen then
        we can use the request number and we don't know the bounds of our air
        lease.
        """
        #TODO remove this assert... it's here for testing
        assert self.lease_id != lease_id

        # So we're gonna check to see if we have a request record for this
        # request number. If we find one, then we can assign the the request
        # object to `self.current_lease`` otherwise we just fall back to the
        # request number.
        if lease_id in self.pending_requests:
            airspace_record: AirspaceRecord = self.pending_requests[lease_id]
            self._current_airspace = airspace_record
            airspace_record.approved_callback(airspace_record.name, airspace_record.airspace, AirLeaseOutcome(approved=True, deadlock=False))
        self._current_lease_id = lease_id
    
    def _cleanup_pending_requests(self, request_number: RequestNumber) -> None:
        # Next we're gonna do a little cleanup. We're gonna remove any pending
        # requests that are less than or equal to the request number. This is
        # because the protocol requires that request numbers always count up. So
        # if there exists a pending request with a smaller request number, then
        # it's no longer relevant. Any time the ground control station receives
        # a request with a smaller request number, it will always deny it.
        outdated_request_numbers = [n for n in self.pending_requests.keys() if n <= request_number]
        for req_number in outdated_request_numbers:
            del self.pending_requests[req_number]
        
        # TODO should we call the callback for these requests? from the
        # perspective of the drone, these requests are no longer relevant. they
        # were probably denied by the ground control station because the drone
        # was out of sync. not because the drone was asking for airspace that
        # isn't available... Right now, I don't think we should call the
        # callback sync errors. It's not the drone's fault. we should just clean
        # up and move on.

    def on_exit_sending_request(self, *args, **kwargs) -> None:
        pass

    def receive_response(self, response: Union[MQTTMessage,str,Dict]) -> None:
        """This runs when we get a response from the ground control station.
        All messages look like this:
        {
            "drone_id": str,
            "request_number": int,
            "current_lease": int,
            "approved": bool,
            "deadlock": bool,
        }

        First we need de-serialize the message
        Then we need to see if self.current_lease is out of sync
        """
        if isinstance(response, MQTTMessage):
            response = response.payload.decode("utf-8")
        if isinstance(response, str):
            response: AirLeaseResponse = json.loads(response)

        msg_request_number = response["request_number"]
        if msg_request_number not in self.pending_requests:
            # this is an outdated message. we must ignore it.
            return
        
        # find the pending request that matches the request number. We need to
        # get this _now_ because we're about to "clean up" the pending requests
        # and this airspace record might be removed. However, we may need to
        # access it later in this method.
        airspace_record = self.pending_requests[msg_request_number]

        # first we're gonna try to re-sync our current air lease knowledge. If
        # the GCS says we have a different air lease, then we must update our
        # field: `lease_id`.
        gcs_lease_id = response["current_lease"] 

        # so if we've made it this far, then we know the GCS is responding to a
        # request that we consider "active" (meaning we have not yet determined
        # its outcome). Now we need to check if our actual lease ID is different
        # from what we think it is. If it is different, then we need to update
        # our data. This could happen if a response message is lost, but This
        # could also happen if the GCS reboots.
        is_lease_id_different = gcs_lease_id != self.lease_id
        if is_lease_id_different:
            self._update_current_lease(gcs_lease_id)
            self._cleanup_pending_requests(gcs_lease_id)

        # next we need to clean up all the pending requests with a request
        # number less than or equal to `msg_request_number`. this helps us
        # ignore any responses that are out of order, duplicated or out of order
        # + duplicated. 
        self._cleanup_pending_requests(msg_request_number)

        # By this point, we have already updated our current lease if applicable. 
        # So, now we just need to process the response message.

        if airspace_record == self.target_airspace:
            outcome = AirLeaseOutcome(
                approved=response["approved"],
                deadlock=response["deadlock"],
            ) 
            if States.IDLE == self.state:
                """
                There is an edge case where we might receive a response that's
                referencing our target air space while we're in the IDLE state.
                This could happen in case a number of requests were in-flight
                and we just happen to receive a response for some of them.
                
                For example, suppose the MQTT connection is "working" but it's
                slow and all of the following is true:

                - the drone sends multiple requests for the same airspace
                  volume.
                - the requests or their responses exist in-flight for an
                  extended period of time.

                In this case, we might get a response for the oldest in-flight
                request that approves the airspace.

                Later we might get a response for one of the younger in-flight
                requests. The ground station would need to deny the younger
                requests because the current_lease would be outdated. (NOTE:
                when the older request was approved, the current_lease would
                have changed so all younger in-flight requests will get denied).
                
                If this were to happen then we would end up here. Specifically
                we would end up receiving a response while we're in the IDLE
                state and the response would be for our target airspace and the
                response would deny the request.

                Therefore we will assert that the outcome is not approved to
                make sure we're only dealing with this known edge case.
                """
                is_denied = not outcome.approved
                assert is_denied, "Edge case detected: We received an approved response for the target airspace while in IDLE state. This should never happen. This is likely a bug. Please report it."
                return
            
            # so this is tricky. There are two ways we can find out that we currently 
            # hold the airspace volume we target.
            # 1. we get a response that directly approves the airspace volume
            # 2. we get a deny response that tells us our current lease is one that maps back to the airspace volume we target
            # so can check if the response directly approves the airspace volume or if our current lease is the one we're looking for.
            if response["approved"] or self.airspace == airspace_record.airspace:
                self.lease_approved()
                return
            elif not is_lease_id_different:               
                airspace_record.denied_callback(airspace_record.name, airspace_record.airspace, outcome)
                # so we only want to enter the lease_denied state if we're denied
                # but not because we're out of sync. If we're out of sync, then
                # we just want to re-sync and try again
                self.lease_denied()
                return
        
        if is_lease_id_different:
            # When the lease_id is different, it likely indicates one of the
            # following scenarios:
            #
            # 1. we are out of sync. We thought our current lease was X but it
            #    was Y.
            #
            # 2. We are in sync, but the GCS received a request that was
            #    in-flight for a while and it's just now catching up.
            #
            # so long as we're not in the IDLE state, we can retry the request.
            #
            # But we don't want to retry if we're in the IDLE state because that
            # means we already hold the airspace volume we want. So there's no
            # need to request anything. In this case it's likely that these
            # denied responses are from outdated messages that were in-flight
            # for a while. It should be safe to ignore them.
            if States.IDLE == self.state:
                # If we're in the IDLE state, then we don't need to retry. This
                # probably means that the GCS is responding to an in-flight
                # request that's no longer relevant.
                return
            else:
                # If the state is not IDLE, then it means we're actively trying
                # to acquire an air lease. In this case, we should retry right
                # away because it likely indicates our pending requests assumed
                # the wrong current lease.  Furthermore this probably means our
                # airspace wasn't even considered. It was just rejected early
                # because we're out of sync. 
                self.retry_request()

    def on_lease_approved(self, *args, **kwargs) -> None:
        # call the callback, cleanup target_airlease
        # self.target_airspace = None
        pass

    def on_tick_idle(self, delta_time: float):
        """We really don't have anything to do in the idle state.
        But we need to have a tick method because the tick events will be
        delivered to this state machine and it's easier to just have a
        no-op here than to have to check if the state is idle before
        calling tick.
        """
        pass

    def on_enter_waiting_response(self, *args, **kwargs) -> None:
        # TODO do something more clever...
        # maybe we should keep track of how long it's been taking to get a
        # response. and we should make the retry timer equal to double the
        # average response time. Maybe we can have a rolling average of the
        # last 10 response times.
        self.state_data = _TimeAccumulator(2.0)

    def on_tick_waiting_response(self, delta_time: float):
        timer: _TimeAccumulator = self.state_data
        timer.update(delta_time)
        if timer.is_time_up():
            self.retry_request()

    def on_exit_waiting_response(self, *args, **kwargs) -> None:
        pass

    def on_tick_disconnected(self, delta_time: float):
        if self.mqtt.is_connected():
            self.connection_restored()

    def on_enter_lease_denied(self, *args, **kwargs) -> None:
        # TODO do something more clever...
        # we need to wait a random number of seconds between 1-3 seconds 
        # before we retry the request.
        timer_value = random.uniform(1.0, 3.0)
        self.state_data = _TimeAccumulator(timer_value)

    def on_tick_lease_denied(self, delta_time: float):
        timer: _TimeAccumulator = self.state_data
        timer.update(delta_time)
        if timer.is_time_up():
            self.retry_request()
    
    def on_exit_lease_denied(self, *args, **kwargs) -> None:
        pass

    def on_exit_receving_response(self, *args, **kwargs) -> None:
        pass

    def on_enter_disconnected(self, *args, **kwargs) -> None:
        pass

    def on_exit_disconnected(self, *args, **kwargs) -> None:
        pass


states = [
    State(
        name=States.IDLE,
        on_enter=["on_enter_idle"],
        on_exit=["on_exit_idle"],
        ignore_invalid_triggers=False,
        final=False,
    ),
    State(
        name=States.SENDING_REQUEST,
        on_enter=["on_enter_sending_request"],
        on_exit=["on_exit_sending_request"],
        ignore_invalid_triggers=False,
        final=False,
    ),
    State(
        name=States.WAITING_RESPONSE,
        on_enter=["on_enter_waiting_response"],
        on_exit=["on_exit_waiting_response"],
        ignore_invalid_triggers=False,
        final=False,
    ),
    State(
        name=States.LEASE_DENIED,
        on_enter=["on_enter_lease_denied"],
        on_exit=["on_exit_lease_denied"],
        ignore_invalid_triggers=False,
        final=False,
    ),
    State(
        name=States.DISCONNECTED,
        on_enter=["on_enter_disconnected"],
        on_exit=["on_exit_disconnected"],
        ignore_invalid_triggers=False,
        final=False,
    ),
]

# The `stable_states` are states that we can be in for a long time. It's possible for the transition to finish leaving us in one of these states. Then as messages arrive, we can process them using Internal transitions. These are transitions that don't change the state of the machine. They just run some callback functions.
# `momentary_states` are states that immediately transition to another state.
stable_states = [
    States.IDLE,
    States.WAITING_RESPONSE,
    States.DISCONNECTED,
    States.LEASE_DENIED,
]

momentary_states = [
    States.SENDING_REQUEST,
]

# Transitions
# Here's how it works:
#   Each transition create a trigger method that initiates the
#   "transition" procedure. The procedure runs four **main** callbacks:
#       1. on_before_<transition_name>
#       2. on_exit_<state_name>
#       3. on_enter_<state_name>
#       4. on_after_<transition_name>
#
#   Note: you can register more than one callback for each of these events, but it's simpler to just register one or none.
#
#   So the best practice is to 
transitions = [
    {
        "trigger": "update_target_airspace_volume", 
        "source": stable_states,
        "dest": States.SENDING_REQUEST,
        "before": "on_update_target_airspace_volume",
    },
    {
        "trigger": "request_sent",
        "source": States.SENDING_REQUEST,
        "dest": States.WAITING_RESPONSE,
    },
        {
        "trigger": "retry_request",
        "source": [States.WAITING_RESPONSE, States.LEASE_DENIED],
        "dest": States.SENDING_REQUEST,
    },
    {
        "trigger": "lost_connection",
        "source": States.SENDING_REQUEST,
        "dest": States.DISCONNECTED,
    },
    {
        "trigger": "connection_restored",
        "source": States.DISCONNECTED,
        "dest": States.SENDING_REQUEST,
    },
    {
        "trigger": "lease_approved",
        "source": [States.WAITING_RESPONSE, States.DISCONNECTED, States.LEASE_DENIED],
        "dest": States.IDLE,
        'after': 'on_lease_approved',
    },
    {
        "trigger": "lease_denied",
        "source": [States.WAITING_RESPONSE, States.DISCONNECTED],
        "dest": States.LEASE_DENIED,
    },
    {
        "trigger": "tick",
        "source": States.IDLE,
        "dest": None,
        "before": "on_tick_idle",
    },
    {
        "trigger": "tick",
        "source": States.WAITING_RESPONSE,
        "dest": None,
        "before": "on_tick_waiting_response",
    },
    {
        "trigger": "tick",
        "source": States.DISCONNECTED,
        "dest": None,
        "before": "on_tick_disconnected",
    },
    {
        "trigger": "tick",
        "source": States.LEASE_DENIED,
        "dest": None,
        "before": "on_tick_lease_denied",
    },
]

def make_model_and_machine(uav_id: str, mqtt: MQTTClient):
    model = AirLeaserModel(uav_id, mqtt)
    machine = Machine(
        model=model,
        states=states,
        initial=States.IDLE,
        transitions=transitions,
    )
    return model, machine
