import json
import math
import queue
from dr_onboard_autonomy.message_senders import RepeatTimer
import rospy
from typing import Any, Dict, Union, TypedDict, Optional, Literal
import traceback

from paho.mqtt.client import MQTTMessage
from tf.transformations import quaternion_multiply
from pyquaternion import Quaternion as Quat

from dr_onboard_autonomy.models import DATUM_REFERENCE, LlaPosition, Quaternion
from dr_onboard_autonomy.models import drone
from dr_onboard_autonomy.models.config import CameraConfig
from dr_onboard_autonomy.briar_helpers import BriarLla, LlaDict
from dr_onboard_autonomy.states import BaseState



class VisionTriggerPosition(TypedDict):
    latitude: float
    longitude: float
    altitude: float
    datum_ref: Literal["ELLIPSOID_WGS84"]


class VisionTriggerQuaternion(TypedDict):
    x: float
    y: float
    z: float
    w: float


class VisionTriggerCameraConfig(TypedDict):
    horizontal_fov: float
    vertical_fov: float


class CalculationParameters(TypedDict):
    """
    CalculationParameters is a TypedDict used for logging and tracing
    the parameters we use for geo-locating objects found by the computer
    vision service.

    This dictionary serves as an internal "blackboard" to capture all necessary parameters required for the geo-locating calculation. It is designed to be serialized to and from JSON, so we can log it, and do offline debuggging and tracing where we re-run the same calculations using real-world inputs (possibly in unit tests).

    **Notes:**
    - **Optional Fields:** Fields marked as `Optional` are intended to be computed from the non-optional fields. For example, `camera_attitude_enu` is optional because it going to be computed from `gimbal_attitude_enu` and `camera_attitude_gimbal`.
    
    - **Coordinate Systems:** Each quaternion field specifies its coordinate system using a naming convention:
        - Fields ending with `_enu` are expressed in the East-North-Up (ENU) coordinate system.
        - the `camera_attitude_gimbal` ends with `_gimbal` to indicate its relative to the gimbal's operating frame.
    """
    # Timestamp in milliseconds, serves as the unique ID for the
    # request. All correspondence related to this request will reference
    # this value, ensuring the receiver knows which request is being
    # discussed.
    timestamp_ms: int 

    x: int # X-coordinate of the image or object
    y: int # Y-coordinate of the image or object
    x_res: int # X-axis resolution (e.g., camera resolution)
    y_res: int # Y-axis resolution (e.g., camera resolution)

    home_position: VisionTriggerPosition
    drone_position: VisionTriggerPosition

    # Attitude of the drone in ENU coordinate system
    drone_attitude_enu: VisionTriggerQuaternion 

    # Orientation of the gimbal's frame in ENU coordinates
    gimbal_attitude_enu: Optional[VisionTriggerQuaternion] 

    # Attitude of the camera in the gimbal's frame
    camera_attitude_gimbal: VisionTriggerQuaternion

    # Attitude of the camera in ENU coordinate system
    camera_attitude_enu: Optional[VisionTriggerQuaternion]

    camera_config: VisionTriggerCameraConfig
    res_topic: str


class Helpers:
    """Collect a bunch of helper functions that are used to serialize and deserialize CalculationParameters
    """
    @staticmethod
    def timestamp(params: CalculationParameters) -> float:
        return params["timestamp_ms"] / 1000

    @staticmethod
    def target_coords(params: CalculationParameters) -> tuple:
        return params["x"], params["y"]
    
    @staticmethod
    def image_res(params: CalculationParameters) -> tuple:
        return params["x_res"], params["y_res"]
    
    @staticmethod
    def fov_h(params: CalculationParameters) -> float:
        return params["camera_config"]["horizontal_fov"]

    @staticmethod
    def lla_tuple(self: VisionTriggerPosition):
        return self["latitude"], self["longitude"], self["altitude"]
    
    @staticmethod
    def from_lla(lla: LlaPosition) -> VisionTriggerPosition:
        lla = lla.to_wgs84_ellipsoid()
        return {
            "latitude": lla.latitude,
            "longitude": lla.longitude,
            "altitude": lla.altitude,
            "datum_ref": lla.datum_ref.name,
        }

    @staticmethod
    def from_dict(lla: LlaDict) -> VisionTriggerPosition:
        return {
            "latitude": lla["latitude"],
            "longitude": lla["longitude"],
            "altitude": lla["altitude"],
            "datum_ref": DATUM_REFERENCE.ELLIPSOID_WGS84.name,
        }
    
    @staticmethod
    def to_quat(q: VisionTriggerQuaternion) -> Quat:
        w, x, y, z = q["w"], q["x"], q["y"], q["z"]
        return Quat(w=w, x=x, y=y, z=z)
    
    @staticmethod
    def to_dr_quat(q: VisionTriggerQuaternion) -> Quaternion:
        return Quaternion(q["x"], q["y"], q["z"], q["w"])

    @staticmethod
    def from_quaternion(q: Union[Quat, Quaternion]) -> VisionTriggerQuaternion:
        return {
            "x": q.x,
            "y": q.y,
            "z": q.z,
            "w": q.w,
        }

    @staticmethod
    def from_camera_config(config: CameraConfig) -> VisionTriggerCameraConfig:
        return {
            "horizontal_fov": config.horizontal_fov,
            "vertical_fov": config.vertical_fov,
        }


class VisionTrigger:
    """Component to trigger a state transition when we receive a vision message.
    """
    
    REQUEST_TOPIC = "dr-onboard/req-location-from-frame-data"
    RESPOSE_TOPIC = "vision/resp-location"

    def __init__(self,
            state: BaseState,
            home: LlaDict,
            gimbal_calculator: 'GimbalQuatCalculator',
            camera_config: CameraConfig,
            allow_transition: bool = True,
            **kwargs
        ):
        """
        Parameters:
            state: The state that this component is attached to
            home: The home position of the drone
            gimbal_calculator: The gimbal calculator to use for finding the gimbal's frame in ENU
            camera_config: The camera configuration. This provides the field of view of the camera.
            allow_transition: If True, the component will trigger a state transition when a found message is received. If False, then this is a passive component that responds to requests from the vision service but it doesn't trigger state transition.
        """
        self.state = state
        self.drone: drone.CopterDrone = state.drone
        self.local_mqtt = state.local_mqtt_client
        self.home = BriarLla.from_dict(home, is_amsl=True)
        
        if allow_transition:
            state.message_senders.add(state.reusable_message_senders.find("vision_found"))
            state.handlers.add_handler("vision_found", self.on_vision)
        
        self.geolocator: Union['GeoLocator', Literal[False]] = kwargs.get('geolocator', False)

        state.message_senders.add(state.reusable_message_senders.find("vision_request"))
        state.handlers.add_handler("vision_request", self.on_request, can_transition=False)

        state.message_senders.add(RepeatTimer("retry_vision_calculations", 1.0))
        state.handlers.add_handler("retry_vision_calculations", self.on_retry_vision_requests, can_transition=False)
        self.request_queue = DoubleBufferQueue()
        self.gimbal_calculator = gimbal_calculator
        self.camera_config = camera_config

    def on_vision(self, _):
        return "found"

    def on_retry_vision_requests(self, _):
        self.request_queue.swap()
        while not self.request_queue.empty():
            msg = self.request_queue.get()
            self.on_request(msg)
    
    def on_request(self, msg):
        mqtt_message: MQTTMessage = msg['data']
        
        # Parse the request
        try:
            request_json: str = mqtt_message.payload.decode('utf-8')
            request = json.loads(request_json)
            # need to make sure it's a dictionary
            if not isinstance(request, dict):
                raise TypeError(f"Expected a dictionary, got {type(request)}")
        except UnicodeDecodeError as e:
            rospy.logerr(f"Could not decode utf-8 string from {mqtt_message.topic}:\n\n```{mqtt_message.payload}\n```")
            rospy.logerr(f"Error: {e}")
            rospy.logerr(traceback.format_exc())
            self.send_error_respnses(reason=f"{e}") # In this case we cannot specify the timestamp so we will send a response with timestamp=0
            return
        except json.JSONDecodeError as e:
            rospy.logerr(f"VisionTrigger: Error decoding JSON message from vision service.")
            rospy.logerr(f"VisionTrigger: Received a bad message from MQTT topic: '{mqtt_message.topic}', Payload: {request_json}")
            rospy.logerr(f"VisionTrigger: Error: {e}")
            rospy.logerr(traceback.format_exc())
            self.send_error_respnses(reason=f"{e}") # In this case we cannot specify the timestamp so we will send a response with timestamp=0
            return
        except TypeError as e:
            rospy.logerr(f"VisionTrigger: Error trying to process request from the vision service: the request is not a dictionary.")
            rospy.logerr(f"VisionTrigger: MQTT topic: '{mqtt_message.topic}', Payload: {request_json}")
            rospy.logerr(f"VisionTrigger: Request: {request}")
            rospy.logerr(f"VisionTrigger: Error: {e}")
            rospy.logerr(traceback.format_exc())
            self.send_error_respnses(reason=f"{e}")
            return
        
        # Extract the timestamp and response topic
        # These two values are required for further error handling
        # and onging communication with the vision service
        res_topic = request.get("res_topic", VisionTrigger.RESPOSE_TOPIC)
        try:
            timestamp_ms = request["timestamp"]
            # From this point forward we can improve our error handling
            # because we know the timestamp and response topic
        except KeyError as e:
            rospy.logerr(f"VisionTrigger: Error trying to process request from the vision service: the request is missing a timestamp.")
            rospy.logerr(f"VisionTrigger: MQTT topic: '{mqtt_message.topic}', Payload: {request_json}")
            rospy.logerr(f"VisionTrigger: Request: {request}")
            rospy.logerr(f"VisionTrigger: Error: {e}")
            rospy.logerr(traceback.format_exc())
            self.send_error_respnses(topic=res_topic, reason=f"This required parameter is missing: {str(e)}")
            return
        
        # Gather all inputs we need for the calculation
        try:
            # inputs from the vision service
            t = timestamp_ms / 1000 # need to convert from ms to s
            x = request["x"]
            y = request["y"]
            x_res = request["x_res"]
            y_res = request["y_res"]

            # Log the request
            if "retries" in msg:
                log_msg = f"VisionTrigger: Attempting to retry request with timestamp: {timestamp_ms}, x: {x}, y: {y}, x_res: {x_res}, y_res: {y_res}"
            else:    
                log_msg = f"VisionTrigger: Received vision request with timestamp: {timestamp_ms}, x: {x}, y: {y}, x_res: {x_res}, y_res: {y_res}"
            rospy.loginfo(log_msg)

            # inputs from the drone
            drone_pos: LlaPosition = self.drone.data.location.lookup_position(t)
            drone_attitude_enu: Quaternion = self.drone.data.attitude.lookup_attitude(t)
            # TODO What do we do when our camera is not on a gimbal?
            # For example, if the camera is fixed to the drone...
            # https://github.com/DroneResponse/DR-OnboardAutonomy/issues/427#issuecomment-2344226847
            camera_attitude_gimbal: Quaternion = self.drone.gimbal.attitude_lookup(t)
        except KeyError as e:
            input_topic = 'dr-onboard/req-location-from-frame-data'
            rospy.logerr(f"Missing key in vision JSON from {input_topic}:\n\n```{request_json}\n```")
            rospy.logerr(f"Error: {e}")
            rospy.logerr(traceback.format_exc())
            self.send_error_respnses(topic=res_topic, timestamp_ms=timestamp_ms, reason=f"Request was missing a required value. This value was missing: {str(e)}")
            return
        except LookupError:
            missing_data = []
            newest_timestamps = []
            oldest_timestamps = []
            if not self.drone.data.location.contains(t):
                t0, tN = self.drone.data.location.time_window()
                newest_timestamps.append(tN)
                oldest_timestamps.append(t0)
                err_msg = self._data_window_error_msg(t, t0, tN)
                missing_data.append(f"drone_position(data_window: {t0} to {tN}, {err_msg})")
            if not self.drone.data.attitude.contains(t):
                t0, tN = self.drone.data.attitude.time_window()
                newest_timestamps.append(tN)
                oldest_timestamps.append(t0)
                err_msg = self._data_window_error_msg(t, t0, tN)
                missing_data.append(f"drone_attitude(data_window: {t0} to {tN}, {err_msg})")
            if not self.drone.gimbal.attitude_contains(t):
                t0, tN = self.drone.gimbal.attitude_time_window()
                newest_timestamps.append(tN)
                oldest_timestamps.append(t0)
                err_msg = self._data_window_error_msg(t, t0, tN)
                missing_data.append(f"gimbal_attitude(data_window: {t0} to {tN}, {err_msg})")
            missing_data_str = ", ".join(missing_data)
            rospy.logwarn(f"VisionTrigger: Error processing request (timestamp={timestamp_ms}) Cannot geo-locate object at t={t} as the following data are missing: {missing_data_str}")
            # find the biggest timestamp 
            most_out_of_date = min(newest_timestamps) # the smallest of the newest timestamps identfies the time window that is furthest in the past and the one that we need to wait for...
            cuttoff_time = max(oldest_timestamps) # the biggest of the oldest timestamps identifies our data cuttoff point. We cannot process any request that's older than this... if this happens, we can't do anything about it. So we will NOT attempt to retry this request.

            num_retries = msg.get("retries", 0)
            # we can retry if:
            #  1. t is newer than our cuttoff time (t > cuttoff_time) means we have not discarded the data we need yet
            #  2. t is less than one second ahead of most_out_of_date... this means the drone data is less than 1 second behind t... so t isn't too far in the future (relative to the drone data). We want to cap this to avoid needing to wait a long time for the drone data to catch up.
            #  3. we haven't retried more than 3 times... we don't want to retry indefinitely
            can_retry = cuttoff_time <= t and t - most_out_of_date < 1.5 and num_retries < 3
            if can_retry:
                msg["retries"] = num_retries + 1
                msg["timestamp"] = timestamp_ms 
                self.request_queue.put(msg)
            else:
                self.send_error_respnses(res_topic, timestamp_ms, reason=f"Drone data is missing and cannot be retrieved. {missing_data_str}")
            return

        camera_params: CalculationParameters = {
            "timestamp_ms": timestamp_ms,
            "x": x,
            "y": y,
            "x_res": x_res,
            "y_res": y_res,
            "home_position": Helpers.from_dict(self.home.ellipsoid.dict),
            "drone_position": Helpers.from_lla(drone_pos),
            "drone_attitude_enu": Helpers.from_quaternion(drone_attitude_enu),
            "gimbal_attitude_enu": None,
            "camera_attitude_gimbal": Helpers.from_quaternion(camera_attitude_gimbal),
            "camera_attitude_enu": None,
            "camera_config": Helpers.from_camera_config(self.camera_config),
            "res_topic": res_topic,
        }
        self.geo_locate_object_from_camera(camera_params)

    def _data_window_error_msg(self, t, t0, tN) -> str:
        """Find out if t is before the data window or after the data window and show how far it is from the data window
        """
        if t < t0:
            # how much time between t and t0
            delta = t0 - t
            # I want to print "detlta" down to the nearest 4 decimal places
            delta = round(delta, 4) 
            return f"Data window error: t={t} is too old and falls outside the data window by {delta} seconds"
        if t > tN:
            # how much time between t and tN
            delta = t - tN
            delta = round(delta, 4)
            return f"Data window error: t={t} is too new and falls outside the data window by {delta} seconds"
        # this shouldn't happen but if it does, lets return an empty string...
        return ""
    
    def geo_locate_object_from_camera(self, parameters: CalculationParameters):
        # Log the parameters as a JSON string that we can easily copy and paste into a unit test
        # First we log the data we've received from sensors and external services
        # These values are not derived from other values
        log_msg = f"VisionTrigger: Finding derived parameters from inputs:\n````\n{json.dumps(parameters)}\n````"
        rospy.loginfo(log_msg)

        
        drone_attitude_enu = Helpers.to_dr_quat(parameters['drone_attitude_enu'])
        is_success, gimbal_attitude_enu = self.gimbal_calculator.find_gimbal_quat(drone_attitude_enu)
        if not is_success:
            self.send_error_respnses(
                topic=parameters["res_topic"],
                timestamp_ms=parameters["timestamp_ms"],
                reason="Error: could not calculate gimbal's frame in ENU. This could happen if (1) we found rotations that are beyond the roll, pitch, and/or yaw limits of the gimbal and/or (2) the resulting gimbal frame is upside down. (the z-axis of the gimbal frame must be aligned with local Earth Up)"
            )
            return

        parameters["gimbal_attitude_enu"] = Helpers.from_quaternion(gimbal_attitude_enu)
        gimbal_attitude_enu = Helpers.to_quat(parameters["gimbal_attitude_enu"])
        camera_attitude_gimbal = Helpers.to_quat(parameters["camera_attitude_gimbal"])

        # NOTE: what comes next is a tricky math detail. 
        # We apply a fixed rotation of 180° around the X axis at the very
        # end because the gimbal uses a forward-right-down frame and our
        # code expects a forward-left-up frame.
        #
        # The gimbal_attitude_enu will get us to the gimbal's operating
        # frame such that: X is aligned with X, Y is aligned with Y, and
        # Z is aligned with Z.
        #
        # However, gimbal_attitude_enu is still forward-left-up. This is
        # acceptable for now because the gimbal itself doesn't
        # differentiate between forward-left-up and forward-right-down.
        # As long as the axes are aligned our optical axis will end up
        # in the right angular position.
        #
        # The camera, on the other hand, does care about the
        # orientation. In the camera frame, we need to ensure that up
        # aligns with up. Therefore, we apply this final rotation to
        # flip the camera right-side up.
        camera_attitude_enu = (gimbal_attitude_enu * camera_attitude_gimbal) * Quat(angle=math.pi, axis=[1, 0, 0])
        parameters["camera_attitude_enu"] = Helpers.from_quaternion(camera_attitude_enu)

        # Now we log all the parameters one last time before we do the calculation
        # At this point we've derived all the values we need to calculate the object's position
        log_msg = f"VisionTrigger: All parameters found. Calculating object position from:\n````\n{json.dumps(parameters, indent=4)}\n````"
        rospy.loginfo(log_msg)
    
        if self.geolocator:
            self.geolocator.submit_calculations(parameters, self.local_mqtt)
            return
    
        try:
            object_lla_ellipsoid = geolocation.geolocate_object_from_camera(
                fov_h=Helpers.fov_h(parameters),
                lla=Helpers.lla_tuple(parameters["drone_position"]),
                image_res=Helpers.image_res(parameters),
                target_coords=Helpers.target_coords(parameters),
                quaternion_gimbal=Helpers.to_dr_quat(parameters["camera_attitude_enu"]),
                ground_alt=parameters["home_position"]["altitude"],
            )
            reason = False
        except geolocation.RayIntersectionError as e:
            is_success = False
            object_lla_ellipsoid = (0, 0, 0)
            rospy.logerr(f"VisionTrigger: Error calculating object position: {e}")
            reason = f"Could not calculate the ray intersection position: {e.message}"
        
        object_lla = LlaPosition(*object_lla_ellipsoid, DATUM_REFERENCE.ELLIPSOID_WGS84).to_amsl()

        output = {
            "timestamp": parameters["timestamp_ms"],
            "lat": object_lla.latitude,
            "lon": object_lla.longitude,
            "alt_amsl": object_lla.altitude,
            "success": bool(is_success),
        }
        if reason:
            rospy.logwarn(f"VisionTrigger: Error calculating object position: {reason}")
            output["reason"] = reason
        res_topic = parameters["res_topic"]
        # Log the output
        log_msg = f"VisionTrigger: Sending geo-location response to the vision service (topic={res_topic}) message:\n```\n"
        log_msg += f"{json.dumps(output, indent=4)}\n"
        log_msg += "```"
        rospy.loginfo(log_msg)
        self.local_mqtt.publish(res_topic, output)

    def send_error_respnses(self, topic=None, timestamp_ms=0, reason:str=""):
            if topic is None:
                topic = VisionTrigger.RESPOSE_TOPIC
            output = {
                "timestamp": timestamp_ms,
                "lat": 0,
                "lon": 0,
                "alt_amsl": 0,
                "success": False,
            }
            
            log_msg = f"VisionTrigger: Sending error response to the vision service"
            if reason:
                output["reason"] = reason
                log_msg += f"; Reason: '{reason}'"
            log_msg += f" (topic={topic}) response message:\n```\n{output}\n```"
            rospy.loginfo(log_msg)
            self.local_mqtt.publish(topic, output)


class TargetPosition:
    """Component to trigger state transition and store a target position when we receive a
    target position
    """
    def __init__(self, state: BaseState):
        self.state = state

        rospy.loginfo("TargetPosition - adding target_position message sender and handler")
        state.message_senders.add(state.reusable_message_senders.find("target_position"))
        state.handlers.add_handler("target_position", self._save_target_position, can_transition=False)
        state.handlers.add_handler("target_position", self._trigger_outcome, can_transition=True)

    def _save_target_position(self, message: Dict[str, Union[str, Any]]):
        """Stores the received target position so it can be used by downstream states
        This is an inactive handler, so it will not trigger a state transition.
        """
        message_position_data = json.loads(message["data"].payload)
        self.state.drone.data.target_position = LlaPosition(
            latitude=message_position_data["latitude"],
            longitude=message_position_data["longitude"],
            altitude=message_position_data["altitude_amsl"],
            datum_ref=DATUM_REFERENCE.AMSL
        )

    def _trigger_outcome(self, _) -> str:
        """Triggers the "target_position" outcome
        This is will trigger a state transition
        """
        return "target_position"


class DoubleBufferQueue:
    """
    DoubleBufferQueue is a double Queue. It has a writer queue and a reader queue. New messages are added to the writer queue and taken from the reader queue. Periodically, we swap the queues and batch process all the messages that have arrived. This is useful for preventing infinite loops when processing vision requests messages that may require additional information. (for more information see the context section below)
    
    **Context:**
    The vision service sends us requests to geo-locate subjects found in the camera frame. When a request arrives, we attempt to process it immediately. However, sometimes we cannot process a request because it depends on missing information.  There are two reasons this might occur:
     
    1. We have not received the missing data yet. If this happens, then we enqueue the message for future processing. We only do this when we expect the missing data to arrive soon.

    2. We discarded the data because it was too old. If the request depends on data we discarded, then we send an error response. We only keep so much recent data in memory, and we can't process requests that depend on data we don't have. 
    
    Periodically, we batch process all the enqueued requests. This batch processing is done on a schedule. Unfortunately, a request might still need information we haven't received yet. When this happens, we will enqueue the request again. But this can cause an infinite loop: (1) we get the request, (2) we attempt to process it, and (3) we enqueue it over and over again.

    To solve this problem, we use the `DoubleBufferQueue` class.

    **How It Works:**
    We have two queues: a **Write Queue** and a **Read Queue**.
    
    First messages accumulate in the Write Queue.
    
    Periodically we swap the queues.

    Then, we process all the messages that have accumulated, which are now in the Read Queue. For each message, we either complete it or enqueue it for later. If we need to enqueue it, we place it in the Write Queue. This ensures that we don't immediately reprocess the same message over and over. We only retry requests in subsequent cycles, effectively preventing infinite processing loops.”
    
    
    """
    def __init__(self):
        self._read_queue = queue.SimpleQueue()
        self._write_queue = queue.SimpleQueue()

    def get(self):
        return self._read_queue.get_nowait()

    def put(self, msg):
        self._write_queue.put_nowait(msg)

    def empty(self) -> bool:
        return self._read_queue.empty()

    def swap(self):
        self._read_queue, self._write_queue = self._write_queue, self._read_queue


from dr_onboard_autonomy.gimbal.geolocator import GeoLocator
from dr_onboard_autonomy.gimbal import geolocation
from dr_onboard_autonomy.gimbal.geolocation import GimbalQuatCalculator