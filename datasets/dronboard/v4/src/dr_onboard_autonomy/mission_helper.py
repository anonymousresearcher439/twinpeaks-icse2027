import copy
import json
from functools import partial
import time
from typing import Callable, Dict, List, Mapping, Optional, Tuple, Union

from dr_onboard_autonomy.models.config import GremsyGimbal, MavlinkGimbalManager
from dr_onboard_autonomy.models.kinematics import DATUM_REFERENCE, LlaPosition
from mavros_msgs.msg import Mavlink
from pymavlink.dialects.v20 import common as mavlink2
from jsonc_parser.parser import JsoncParser
import rospy
import smach

from dr_onboard_autonomy.models import DRConfig, Gimbal, MQTTConfig

_RELATIVE_ALTITUDE = "relative_altitude"
_ALTITUDE = "altitude"


def parse_jsonc(json_str: str) -> Dict:
    """Parse a JSON string into a dictionary.
    We're using the JsoncParser to parse the JSON string.

    This way we can add comments in the JSON string.
    """
    return JsoncParser.parse_str(json_str)


def _replace_rel_alt(obj: Dict, home_altitude: float):
    """Helper function for transform_relative_altitude

    This recursively searches through the mission. It replaces every
    property named `relative_altitude` with a property named `altitude`.

    To find the altitude value, it uses the formula:
        altitude = relative_altitude + home_altitude

    Args:
        obj (dict): The object to search through. This object is changed in place

        home_altitude (float): The altitude of the home position in meters AMSL
    """
    if _RELATIVE_ALTITUDE in obj.keys():
        obj[_ALTITUDE] = obj[_RELATIVE_ALTITUDE] + home_altitude
        del obj[_RELATIVE_ALTITUDE]
    for prop in obj.keys():
        if isinstance(obj[prop], dict):
            _replace_rel_alt(obj[prop], home_altitude)
        elif isinstance(obj[prop], list):
            for item in obj[prop]:
                if isinstance(item, dict):
                    _replace_rel_alt(item, home_altitude)


def _replace_relative_altitude_with_altitude(obj: Dict):
    """Helper function for transform_relative_altitude

    Replace the `relative_altitude` property with an `altitude` property.
    Take the value of the `relative_altitude` property and assign it to the
    `altitude` property. Then delete the `relative_altitude` property.

    These changes are made in place.
    """
    if _RELATIVE_ALTITUDE in obj.keys():
        obj[_ALTITUDE] = obj[_RELATIVE_ALTITUDE]
        del obj[_RELATIVE_ALTITUDE]


def _move_prop(obj: Dict, from_prop: str, to_prop: str):
    """Helper function for transform_relative_altitude

    Move the property named `from_prop` to a property named `to_prop`.

    If `to_prop` already exists, then replace the value of `to_prop`
    with the value from `from_prop`.

    Last delete the `from_prop` property.

    These changes are made in place.
    """
    if from_prop not in obj.keys():
        return
    obj[to_prop] = obj[from_prop]
    del obj[from_prop]


def _is_state_match(state: Dict, cls_name: str) -> bool:
    """Helper function for transform_relative_altitude
    This returns True if the type of state matches the given cls_name, and
    False otherwise
    """
    assert cls_name is not None

    if "class" in state:
        return state["class"] == cls_name

    # If the state doesn't have a class property, then we need to check
    # the name property. This is the legacy way of specifying the class.
    return state.get("name") == cls_name


def process_altitudes(original_mission: Dict, home_altitude: float) -> Dict:
    """Preprocess a mission to update altitude arguments.

    Find every object with a `relative_altitude` property, and replace it
    with an `altitude` property. The value given to the `altitude` property
    depends on the type of object we're changing.

    For most types of states, we calculate the `altitude` value with the formula:
        altitude = home_altitude + relative_altitude

    After we set the altitude property we then delete the `relative_altitude`
    property.

    For the Takeoff state and FlyWaypoints state, we handle them as special cases

    For Takeoff, the `altitude` arg is always interpreted as a relative altitude.
    So if the Takeoff state has a `relative_altitude` property, then we need to
    move it to an `altitude` property. We then delete the `relative_altitude`
    property.

    For the FlyWaypoints state, there is a discrepancy between what the
    ground control station expects and what the FlyWaypoints state expects.
    The ground control station expects the `altitude` arg to be interpreted as a
    relative altitude. But the FlyWaypoints state expects the altitude to be
    above sea level. So for the FlyWaypoints state we update the `altitude`
    property like it's a relative altitude. Specifically we update the `altitude`
    property using the formula:
        altitude = home_altitude + altitude

    Furthermore if the FlyWaypoints state has a `relative_altitude` arg, then we
    treat it like a non-special case.

    If an object has both an `altitude` and `relative_altitude` property, then
    undefined behavior...

    Args:
        original_mission (dict): The original mission

        home_altitude (float): The altitude of the home position in meters AMSL

    Returns:
        dict: The transformed mission. This mission is a copy of the original with
              the changes described above. (The original mission is not modified)
    """

    mission = copy.deepcopy(original_mission)
    # Before we conduct a straightforward recursive search through the mission,
    # converting every relative_altitude into an altitude, we first need to
    # address two special cases.
    #
    # the Takeoff state and FlyWaypoints state are both special cases
    # because their `altitude` arg is always interpreted as a relative altitude.
    # So in these cases we need to add an `altitude` arg with the value of the
    # `relative_altitude`. Then we can delete the `relative_altitude`
    for state in mission["states"]:
        if "args" not in state:
                continue

        # Raise an error if we have both an altitude and relative_altitude property...
        # This is undefined behavior
        if "altitude" in state["args"] and "relative_altitude" in state["args"]:
            rospy.logerr("Found both an altitude and relative_altitude property in a state. This is undefined behavior")
            raise ValueError(f"Cannot initialzie state when both an altitude and relative_altitude argument are present. This is undefined behavior. State = {state}, Mission = {original_mission}")

        if _is_state_match(state, "Takeoff"):
            # If Takeoff.args has a `relative_altitude` property then we
            # need to add an `altitude` property with the same value,
            # then delete the `relative_altitude` property
            _move_prop(state["args"], from_prop=_RELATIVE_ALTITUDE, to_prop=_ALTITUDE)

        if _is_state_match(state, "FlyWaypoints"):
            # If FlyWaypoints then we have a special case. The ground control station expects it's altitude arg to always be interpreted as a relative altitude but the FlyWaypoints implementation expects an altitude above sea level.
            # So if we have an altitude, then we assign it's value to a relative_altitude property then delete the altitude property... so basically the opposite of what we're doing in the Takeoff state...
            # After doing so, we can then treat the FlyWaypoints state like a non-special case
            assert "args" in state, "FlyWaypoints state must have args"
            assert "waypoints" in state["args"], "FlyWaypoints state must have waypoints"

            waypoints = state["args"]["waypoints"]
            if isinstance(waypoints, dict):
                the_waypoint = waypoints # we have just a single waypoint
                _move_prop(the_waypoint, from_prop=_ALTITUDE, to_prop=_RELATIVE_ALTITUDE)

            elif isinstance(waypoints, list):
                for waypoint in waypoints:
                    _move_prop(waypoint, from_prop=_ALTITUDE, to_prop=_RELATIVE_ALTITUDE)

    # Now we can do a straightforward recursive search through the mission
    _replace_rel_alt(mission, home_altitude)
    return mission


def find_custom_outcomes(task_spec: dict)-> List[str]:
    """Find all the custom outcomes specified in the task.
    We search through all transitions in the task to find the transitions that are marked as task outcomes.

    A transition object usually looks like this:
    ```json
    {
        "target": "StateName",
        "condition": "State Outcome"
    }
    ```

    If the transition object has an additional property `is_task_outcome` set to true, then we consider the `target` property to be a custom outcome for the state machine.

    Here's an example:
    ```json
    {
        "target": "custom outcome",
        "condition": "State Outcome",
        "is_task_outcome": true
    }
    ```

    Note if a `target` property is not provided, then we will copy the `condition` property to the `target` property.

    The `condition` property is the outcome of the state.
    The `target` property becomes the outcome of the state machine.
    """
    custom_outcomes = set()
    for state in task_spec.get("states", []):
        for transition in state.get("transitions", []):
            if transition.get("is_task_outcome", False):
                target = transition.get("target", transition["condition"])
                transition["target"] = target
                custom_outcomes.add(target)

    return list(custom_outcomes)


class MissionBuilder:
    """
    The MissionBuilder class is for reading missions and initializing state machines.

    A drone mission is represented as a JSON string following certain rules. The MissionBuilder
    class aids in reading this mission and creating an event-driven state machine to carry it out.

    The states in our state machines depend on various objects such as the message senders (that provide states with input data), the drone object (for communicating with the flight controller), and more. The MissionBuilder class
    helps in initializing some of these dependencies. It also bundles them together in one place.

    The main functionalities of the MissionBuilder class are:
    1) Bundling together all dependencies of the states and aiding initializing some of these dependencies.
    2) Providing methods to help with initializing the states.
    3) Providing the `build_mission` method that takes a mission JSON string and returns a fully
        initialized state machine ready to run the mission.
    """

    def __init__(self,
                 uav_id: str,
                 mqtt_client: 'MQTTArg',
                 drone_config: DRConfig,
                 local_mqtt_client: 'MQTTArg' = "mqtt_local",
                 state_factory: Optional['StateFactoryFunction'] = None,
                 reusable_message_senders: Optional['ReusableMessageSenders'] = None,
                 drone: Optional['CopterDrone'] = None,
                 air_lease_protocol_fsm: Optional['AirLeaserModel'] = None,
                 heartbeat_handler: bool = False,
                 data: Optional[Dict] = None,
                 home: Optional['BriarLla'] = None,
                 performance_analysis: bool = False,
                 system_id: Optional[int] = 1,
                 mqtt_cloud: Optional['MQTTArg'] = None,
                 ):
        """
        Initialize the MissionBuilder instance.

        Args:
            uav_id (str): The unique identifier of the UAV. Also known as the
                UAV name.

            mqtt_client (MQTTArg): The MQTT client for communication with the
                ground control station. Can be a string, an MQTTClient instance
                or None. If a string is provided, an MQTTClient instance will be
                initialized. This string should be the hostname of the MQTT
                broker and this will use the default port of 1883 for the
                connection. If None is provided, then the MQTT client will be
                initialized using the MQTTConfig data in the DRConfig object if
                possible.

            local_mqtt_client (MQTTArg): The MQTT client for communication with
                local processes. Can be a string, an MQTTClient instance, or None.
                If a string is provided, an MQTTClient instance will be initialized.
                This string should be the hostname of the MQTT broker and this will use
                the default port of 1883 for the connection. If None is provided, then
                the MQTT client will be initialized using the MQTTConfig data in the
                DRConfig object if possible.

            state_factory (StateFactoryFunction, optional): A function that returns
                all the `kwargs` needed to add a state to any smach.StateMachine
                instances this creates. If not provided the default value is taken
                from the `state_factory` module. Specifically the `init_state`
                function is used.

            reusable_message_senders (ReusableMessageSenders, optional): A set of
                reusable message senders. If not provided, it will be
                initialized along with all the message senders.

            drone (MAVROSDrone, optional): The MAVROSDrone object for controlling
                the drone. If not provided, it will be initialized.

            air_lease_service (AirLeaseService, optional): The AirLeaseService
                object for sending air leasing messages. If not provided, it
                will be initialized.

            heartbeat_handler (bool, optional): A flag indicating whether to
                handle heartbeats. This value is used when initalizing states.
                When `heartbeat_handler` is set to True, the state must receive
                a heartbeat from the ground. Tf it doesn't, the drone will take
                emergency action.

            data (Dict, optional): A dictionary for inter-state communication.
                If not provided, an empty Dict will be initialized.

            home (BriarLla, optional): The home position of the drone. If not
                provided, it will be read from the drone's GPS during setup.
                This is especially useful for testing.

            performance_analysis (bool, optional): A flag indicating whether to
                measure performance metrics during the mission. This value is
                used when initializing states. If set to True, then the state
                will use a specialized message_queue that records performance
                data. If set to False, then the state will use the ordinary
                type of message_queue. The default value is False.

            system_id (int, optional): The system ID of the drone. This is used
                when initializing the drone. All MAVLink messages sent to the
                drone will be addressed to this system ID. The default value is 1.
                Note: this system_ID must match the system_ID that mavros uses,
                and this system ID must match the system ID that the drone uses.

            mqtt_cloud (MQTTArg): The MQTT client for communication with the cloud.
                Can be a string, an MQTTClient instance, or None. If a string is provided,
                an MQTTClient instance will be initialized. This string should be the hostname
                of the MQTT broker and this will use the default port of 1883 for the connection.
                If None is provided, then the MQTT client will be initialized using the MQTTConfig
                data in the DRConfig object if possible.
        Note:
            The optional arguments are optional to facilitate testing with mock
            instances. In production, all optional arguments should _not_ be
            provided (unless you know what you're doing).

            If you do provide optional arguments, then you must provide all of them.
            Or you'll need to understand how the various arguments depend on each
            other and how they're used. For example, if you provide
            reusable_message_senders, but you don't provide MQTTClient instances,
            then the reusable_message_senders might be missing the message senders
            that communicate with MQTT.
        """
        self._drone = drone
        self._drone_config = drone_config
        self._system_id = system_id
        self._uav_id = uav_id

        self._setup_tasks = [] # note this is a list of functions that need to be called during setup
        # The order we add tasks to this list is important. In general later tasks depend on earlier
        # tasks.

        self._is_setup_done = False

        self.state_factory: 'StateFactoryFunction' = _default_state_factory
        if state_factory:
            self.state_factory = state_factory

        # drone config based initialization - start
        self._gimbal_init_map: Mapping[str, Callable[..., Gimbal]] = {
            "gremsy" : self._init_gimbal_gremsy,
            "mavlink_gimbal_manager" : self._init_gimbal_mavlink_gimbal_manager
        }

        def _register_takeoff_state_ardupilot():
            """Used for drone config driven initialization to initialize register the takeoff state
            specific to an ardupilot copter fcu
            """
            from dr_onboard_autonomy.state_factory import (
                overwrite_state_registration,
                _registry_data
            )
            cls, transitions = _registry_data.find("TakeoffArdupilot")
            overwrite_state_registration("Takeoff", cls, transitions)

        def _register_takeoff_state_px4():
            """Used for drone config driven initialization to initialize register the takeoff state
            specific to a px4 copter fcu
            """
            from dr_onboard_autonomy.state_factory import (
                overwrite_state_registration,
                _registry_data
            )
            cls, transitions = _registry_data.find("TakeoffPx4")
            overwrite_state_registration("Takeoff", cls, transitions)

        fcu_init_map = {
            "ardupilot": (self._init_fcu_ardupilot, _register_takeoff_state_ardupilot),
            "px4": (self._init_fcu_px4, _register_takeoff_state_px4)
        }

        gimbal_calculator = self._init_gimbal_calculator()

        for init_func in fcu_init_map[drone_config.fcu.type]:
            init_func()
        # drone config based initialization - end

        if data is None:
            data = {}
        
        mqtt_client = self._init_mqtt(uav_id, mqtt_client, drone_config.mqtt)
        assert mqtt_client is not None, "GCS MQTT client must be initialized"

        local_mqtt_client = self._init_mqtt(uav_id, local_mqtt_client, drone_config.mqtt_local)
        assert local_mqtt_client is not None, "GCS local MQTT client must be initialized"

        mqtt_cloud = self._init_mqtt(uav_id, mqtt_cloud, drone_config.mqtt_cloud)

        if air_lease_protocol_fsm is None:
            air_lease_protocol_fsm, air_lease_machine = airlease.make_model_and_machine(uav_id, mqtt_client)

        if reusable_message_senders is None:
            reusable_message_senders = ReusableMessageSenders()
            # reusable_message_senders.populate_all(uav_id, mqtt_client, local_mqtt_client)
            init_msg_senders = partial(reusable_message_senders.populate_all, uav_id, mqtt_client, local_mqtt_client, mqtt_cloud)

            self._setup_tasks.append(init_msg_senders)

        self.mandatory_args = {
            "air_lease_protocol_fsm": air_lease_protocol_fsm,
            "drone": self._drone,
            "local_mqtt_client": local_mqtt_client,
            "mission_builder": self,
            "mqtt_client": mqtt_client,
            "reusable_message_senders": reusable_message_senders,
            "uav_id": uav_id,
            "data": data,
            "heartbeat_handler": heartbeat_handler,
            "gimbal_calculator": gimbal_calculator,
            "camera_config": drone_config.camera,
        }
        if mqtt_cloud:
            # If we have a cloud MQTT client, add it to the mandatory args. This
            # prevents any mission-provided argument from overwriting it. By
            # putting mqtt_cloud in mandatory_args, we prevent users from
            # accidentally "shadowing" the 'mqtt_cloud' argument.
            self.mandatory_args["mqtt_cloud"] = mqtt_cloud
        
        # Add SADE identification information
        if drone_config.drone_registration_id is not None:
            self.mandatory_args["drone_registration_id"] = drone_config.drone_registration_id
        if drone_config.pilot_id is not None:
            self.mandatory_args["pilot_id"] = drone_config.pilot_id
        if drone_config.owner_id is not None:
            self.mandatory_args["owner_id"] = drone_config.owner_id
        if drone_config.drone_model is not None:
            self.mandatory_args["model_name"] = drone_config.drone_model

        if home:
            self._update_home_position(home)
        else:
            self.mandatory_args.update({
                "home": None, # Derived from all of the above
                "home_altitude_offset": None, # Derived from all of the above
            })
            # these tasks are needed to set the "home" and "home_altitude_offset" values in mandatory_args
            # First, we need to wait for the connection to mavros.
            # Second, we need to wait for the connection to the flight control unit.
            # Third, do some one-time setup: configure some FCU parameters and tell the FCU to start sending MAVLink messages.
            # Last, we need to read the home position from the drone's GPS, and set the system clock (using GPS time).
            self._setup_tasks.extend([
                open_video_metadata_file,
                self._await_mavros,
                self._await_fcu,
                self._drone.start,
                self._read_home_position,
                self._set_system_clock,
            ])
        self.fallback_args = {
            "performance_analysis": performance_analysis,
        }


    def _init_gimbal_gremsy(self):
        """Used for drone config driven initialization to initialize a gremsy gimbal
        """
        gimbal_axes = GimbalAxes.NONE
        if self._drone_config.gimbal.pitch:
            gimbal_axes = gimbal_axes | GimbalAxes.PITCH
        if self._drone_config.gimbal.roll:
            gimbal_axes = gimbal_axes | GimbalAxes.ROLL
        if self._drone_config.gimbal.yaw:
            gimbal_axes = gimbal_axes | GimbalAxes.YAW

        return GimbalGremsy(
            controller=MavlinkNode(
                system_id=self._system_id,
                component_id=mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER,
            ),
            gimbal_axes=gimbal_axes
        )


    def _init_gimbal_mavlink_gimbal_manager(self):
        """Used for drone config driven initialization to initialize a mavlink gimbal manager
        """
        gimbal_axes = GimbalAxes.NONE
        if self._drone_config.gimbal.pitch:
            gimbal_axes = gimbal_axes | GimbalAxes.PITCH
        if self._drone_config.gimbal.roll:
            gimbal_axes = gimbal_axes | GimbalAxes.ROLL
        if self._drone_config.gimbal.yaw:
            gimbal_axes = gimbal_axes | GimbalAxes.YAW

        return NewGimbalManager(
                mavlink_sender=MavlinkSender(
                    system_id=self._system_id,
                    component_id=mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER,
                    mavlink_pub=rospy.Publisher("mavlink/to", Mavlink, queue_size=1)
                ),
                controller=MavlinkNode(
                    system_id=self._system_id,
                    component_id=mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER,
                ),
                gimbal_axes=gimbal_axes,
                gimbal_manager=MavlinkNode(
                    system_id=self._system_id,
                    component_id=mavlink2.MAV_COMP_ID_AUTOPILOT1,
                )
            )


    def _init_fcu_ardupilot(self):
        """Used for drone config driven initialization to initialize the the ardupilot copter
        mavros drone object
        """
        get_gimbal_class: Gimbal = self._gimbal_init_map[self._drone_config.gimbal.type]

        if not self._drone:
            self._drone = ArdupilotCopterMavrosDrone(
                uav_name=self._uav_id,
                mavlink_node=MavlinkNode(
                    system_id=self._system_id,
                    component_id=mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER
                ),
                gimbal=get_gimbal_class()
            )

    def _init_gimbal_calculator(self):
        config = self._drone_config

        # So we want a three axis gimbal calculator if any of the following are true:
        init_three_axis = [
        # 1. If the gimbal is not specified
            not config.gimbal,
        # 2. If the gimbal is a MavlinkGimbalManager
            isinstance(config.gimbal, MavlinkGimbalManager),
        # 3. if it's a GremsyGimbal and the axes are not specified
            isinstance(config.gimbal, GremsyGimbal) and config.gimbal.axes is None,
        # 4. if it's a GremsyGimbal and the axes are specified as three_axis
            isinstance(config.gimbal, GremsyGimbal) and config.gimbal.axes == "three_axis",

        ]
        if any(init_three_axis):
            rospy.logwarn("This geo-location calculation is not implemented: ThreeAxisGimbalQuatCalculator")
            return ThreeAxisGimbalQuatCalculator()
        # if we have a two axis gimbal...
        two_axes_configs = ["roll_first", "pitch_first"]
        if isinstance(config.gimbal, GremsyGimbal) and config.gimbal.axes in two_axes_configs:
            return TwoAxisGimbalQuatCalculator.build_from_config(config)
        if self._drone_config.gimbal.type == "fixed_camera":
          rospy.logwarn("This geo-location calculation is not implemented: FixedCameraQuatCalculator")
          return FixedCameraQuatCalculator.build_from_config(config)




    def _init_fcu_px4(self):
        """Used for drone config driven initialization to initialize the the px4 copter
        mavros drone object
        """
        get_gimbal_class: Gimbal = self._gimbal_init_map[self._drone_config.gimbal.type]

        if not self._drone:
            self._drone = Px4CopterMavrosDrone(
                uav_name=self._uav_id,
                mavlink_node=MavlinkNode(
                    system_id=self._system_id,
                    component_id=mavlink2.MAV_COMP_ID_ONBOARD_COMPUTER
                ),
                gimbal=get_gimbal_class()
            )

    def _init_mqtt(self, uav_id: str, mqtt_arg: 'MQTTArg', config: Optional[MQTTConfig]) -> Optional['MQTTClient']:
        """
        Initializes the MQTT client based on the provided argument.

        If the argument is a string, it initializes an MQTTClient with that hostname.
        If it's already an MQTTClient instance, it returns it as is.
        If it's None, it returns None.
        """
        if isinstance(mqtt_arg, MQTTClient):
            return mqtt_arg
        
        if mqtt_arg is None and config is None:
            return None
        
        if isinstance(mqtt_arg, str):
            mqtt_client = MQTTClient(uav_id, mqtt_arg)
        else:
            mqtt_client = MQTTClient.from_config(uav_id, config)
        
        mqtt_client.connect()
        self._setup_tasks.append(mqtt_client.connected_event.wait)

        return mqtt_client

    def setup(self):
        """
        Runs all the setup tasks.
        """
        for task in self._setup_tasks:
            task()
        self._is_setup_done = True

    def build_kwargs(self, mission_args: Dict={}) -> Dict:
        """
        Constructs a dictionary of keyword arguments (kwargs) for object initialization.

        This method merges three sources of arguments: mandatory arguments, JSON-sourced arguments (mission_args),
        and fallback arguments. The mandatory arguments have the highest priority, followed by the JSON-sourced arguments,
        and finally the fallback arguments. If a key exists in both the mission_args and mandatory_args,
        the value from mandatory_args will be used.

        Parameters:
        mission_args (Dict): A dictionary of arguments sourced from JSON. These arguments are considered
                            for inclusion in the kwargs unless they conflict with mandatory arguments.

        Returns:
        Dict: A dictionary of kwargs that combines the arguments from the fallback_args, mission_args,
            and mandatory_args, with precedence given in that order.
        """
        if not self._is_setup_done:
            rospy.logwarn("MissionBuilder.setup() has not been called. Calling it now.")
            self.setup()
        return self._build_kwargs_internal(mission_args)

    def _build_kwargs_internal(self, mission_args: Dict) -> Dict:
        # Start with the fallback arguments
        kwargs = self.fallback_args.copy()

        # Update with JSON arguments, if they don't override mandatory arguments
        for key, value in mission_args.items():
            if key not in self.mandatory_args:
                kwargs[key] = value

        # Finally, ensure mandatory arguments are set and have precedence
        kwargs.update(self.mandatory_args)

        return kwargs

    def _read_home_position(self):
        pos = self.read_home_position()
        self._update_home_position(pos)
        self._build_geolocator(pos)

    def read_home_position(self):
        """
        Reads the home position from drone's GPS and updates the mandatory_args dictionary.

        This method will block until the home position is read.

        POSTCONDITION
        the mandatory_args dictionary will have the following keys defined:
            home: an LLA dict with the altitude specified in meters AMSL
            home_altitude_offset: a float
        """
        kwargs = self._build_kwargs_internal({})
        position_reader = ReadPosition(**kwargs)
        outcome = position_reader.execute(None)
        if outcome != "succeeded":
            rospy.loginfo("could not read drone position")
            rospy.signal_shutdown("could not read drone position")
            raise Exception(f"could not read position. position_reader outcome: '{outcome}'")

        pos: BriarLla = position_reader.return_channel.get()
        return pos

    def reset_home_position(self):
        rospy.loginfo("Resetting home position")
        pos = self.read_home_position()
        self._update_home_position(pos)

    def _update_home_position(self, pos: 'BriarLla'):
        lat, lon, alt = pos.amsl.lla.lat, pos.amsl.lla.lon, round(pos.amsl.lla.altitude, 2)
        rospy.loginfo(f"Home Position: Lla{lat, lon, alt}")
        altitude = pos.amsl.lla.altitude
        alt_str = format(altitude, ".2f")
        rospy.loginfo(f"Home Altitude (AMSL): {alt_str}")
        self.mandatory_args.update({
            "home": pos.amsl.dict,
            "home_altitude_offset": altitude,
        })

    def _build_geolocator(self, pos: 'BriarLla'):
        """Given the current position, init the GeoLocator object
        """
        rospy.loginfo("Building GeoLocator object")
        home = LlaPosition(pos.amsl.lla.lat, pos.amsl.lla.lon, pos.amsl.lla.altitude, datum_ref=DATUM_REFERENCE.AMSL)
        geolocator = GeoLocator(home)
        self.mandatory_args.update({
            "geolocator": geolocator
        })
        t = time.time()
        if self._drone_config.computer_vision:
            use_bvhtree = self._drone_config.computer_vision.use_bvhtree
        else:
            use_bvhtree = False
        rospy.loginfo(f"Starting GeoLocator object. use_bvhtree set to {use_bvhtree}")
        geolocator.start(use_bounding_volumes=use_bvhtree)
        rospy.loginfo(f"GeoLocator object started in {round(time.time() - t,1)} seconds")

    def _set_system_clock(self):
        """
        Sets the system clock to the current time.
        """
        kwargs = self._build_kwargs_internal({})
        time_setter = SetSystemClock(**kwargs)
        outcome = time_setter.execute(None)
        if outcome not in ["succeeded", "skipped"]:
            rospy.loginfo("Error when setting the system clock")
            rospy.signal_shutdown("could not set system clock")
            raise Exception(f"could not set system clock. set_system_clock outcome: '{outcome}'")
        if outcome == "skipped":
            rospy.logwarn("Could not set the system clock using GPS. Continuing anyway...")




    def _await_mavros(self):
        """
        Waits for mavros to come online.

        This is useful for when the program is initalizing.
        """
        kwargs = self._build_kwargs_internal({})
        awaiter = AwaitMAVROS(**kwargs)
        outcome = awaiter.execute(None)
        if outcome != "succeeded":
            rospy.loginfo("Could not connect to MAVROS")
            rospy.signal_shutdown("Could not connect to MAVROS")
            raise Exception(f"Could not connect to MAVROS. AwaitMAVROS outcome: '{outcome}'")

    def _await_fcu(self):
        """
        Waits for the Flight Control unit to come online.

        This is useful for when the program is initalizing and we need to wait for the FCU to be
        ready.
        """
        kwargs = self._build_kwargs_internal({})
        awaiter = AwaitFCU(**kwargs)
        outcome = awaiter.execute(None)
        if outcome != "succeeded":
            rospy.loginfo("Could not connect to FCU")
            rospy.signal_shutdown("Could not connect to FCU")
            raise Exception(f"Could not connect to FCU. AwaitFCU outcome: '{outcome}'")

    def build_mission(self, mission_json: str) -> smach.StateMachine:
        """
        Given a JSON string, this will build a smach.StateMachine

        Args:
            mission_json (str): A JSON string that specifies the mission

        Returns:
            smach.StateMachine: The mission
        """
        if not self._is_setup_done:
            rospy.logwarn("MissionBuilder.setup() has not been called. Calling it now.")
            self.setup()
        rospy.loginfo(f"Building mission:\n```\n{mission_json}\n```")
        mission = parse_jsonc(mission_json)
        mission = process_altitudes(mission, self.mandatory_args["home_altitude_offset"])

        state_machine = smach.StateMachine(outcomes=["failure", "mission_completed"])
        with state_machine:
            rospy.logdebug("Adding states to smach")
            rospy.logdebug("Adding states from mission_spec to smach")
            for i in mission["states"]:
                assert isinstance(i, dict)
                i: Dict = i

                state_name = i["name"]
                class_name = i.get("class", None)
                if class_name:
                    rospy.loginfo(f"Add state: '{state_name}' class: {class_name} to smach")
                else:
                    rospy.loginfo(f"Add state: '{state_name}' to smach")


                if state_name in ["MISSION_PREPARATION", "MissionPreparation"]:
                    rospy.logdebug("Skipped MissionPreparation")
                    continue

                mission_args = i.get("args", {})
                mission_args['name'] = state_name
                state_args = self.build_kwargs(mission_args)

                transition_spec = i["transitions"]

                smach_args = self.state_factory(state_name, transition_spec, state_args, class_name)
                smach.StateMachine.add(**smach_args)

                rospy.loginfo(f"Done adding state: '{state_name}'")
            rospy.logdebug("Done adding states from mission_spec")

            rospy.logdebug("Adding safety-related  states")
            safety_related_states = [
                "AbortHover",
                "Failsafe",
                "Rtl",
                "ReturnToRecharge"
            ]
            for state_name in safety_related_states:
                state_args = self.build_kwargs({})
                smach_args = self.state_factory(state_name, [], state_args)
                smach.StateMachine.add(**smach_args)
            rospy.logdebug("Finished adding states to smach")

        return state_machine

    def build_task(self, task_json: str) -> Tuple[smach.StateMachine, List[str]]:
        """
        Given a JSON string, this will build a smach.StateMachine

        Args:
            task_json (str): A JSON string that specifies the task

        Returns:
            smach.StateMachine: The task
            List[str]: The custom outcomes for the task. If the task finishes with one of these outcomes, then the task had an ordinary ending.
        """

        if not self._is_setup_done:
            rospy.logwarn("MissionBuilder.setup() has not been called. Calling it now.")
            self.setup()

        """
        How we Initialize a State machine for a Task:

        - The 'task machine' is a specialized state machine used for executing partial-missions. We call these partial-missions: 'tasks', within the overarching drone mission.
        - Each task defines its own set of states and outcomes, and it resembles a mission. A task specifies the same types of states as a mission.
        - Unlike the main mission state machine, the task machine does not initialize or include safety-related states. Safety states are exclusive to the overarching mission state machine.
        - The task machine, however, is fully embedded within a mission state machine. The mission's execute method calls the task machine's execute method. This is what it means for a task to be "embedded" in a mission.

        If during a task, a situation arises where we need to transition to a safety-related state (for example, the pilot takes control with the RC), then this is what happens:

        1. the task machine will _terminate_ with the safety-related outcome.
        2. the mission state machine will see this outcome and transition to the appropriate safety state.

        Tasks have an additional feature: You can cancel a task. This is normal and expected. When you cancel a task, the task machine will terminate with the "task_canceled" outcome. The mission state machine will see this outcome and proceed. For example, the mission might ask for the next task or it might do something else. Overall the mission state machine will continue to execute. And that's what makes canceling a task different from aborting a mission. In contrast, aborting a mission is a significant, atypical event. It is not a routine occurrence and signifies the end of the mission's execution. At this point, the software pilot's role concludes and the operations team assumes control.
        """

        # To start we will define the terminal outcomes for the task machine
        # We start with the standard outcomes that all tasks should recognize
        task_outcomes = [
            "success", # when the task is done
            "task_canceled", #  the task was in progress but was interrupted before it could complete. This is different from "abort" because "abort" is a safety related state. Abort indicates that we need to stop the entire mission because of an emergency. This is not an emergency. This just means something interrupted the task before it could complete.
            "error", # when the program must exit immediately (e.g. we pressed ctrl+c)
            "failsafe", # when a pilot takes control using the RC transmitter
            "abort", # when we receive the abort message from GCS. This aborts the entire mission (not just the current task). The drone goes into a hover state and waits for further instructions.
            "rtl", # this is a placeholder for emergency RTL. This aborts the entire mission (not just the current task). We want the drone to RTL instead of hover. But as of now this is just a placeholder and we just abort the mission and hover
            "low_battery", # when the battery is low and we must abandon the task and return to recharge
        ]
        # Since the task machine is embedded in the mission machine, we can
        # depend on the mission machine to handle safety-related procedures.
        # Therefore Tasks don't have any safety related states. Instead they
        # have safety related outcomes. When a safety related outcomes is
        # reached, the task machine will exit and the mission will respond
        # accordingly.
        safety_related_states = []
        rospy.loginfo(f"Building task:\n```\n{task_json}\n```")
        task_spec = parse_jsonc(task_json)
        # next we need to find the custom outcomes that the task machine should recognize
        custom_outcomes = find_custom_outcomes(task_spec)
        task_outcomes.extend(custom_outcomes)

        task_spec = process_altitudes(task_spec, self.mandatory_args["home_altitude_offset"])
        task_id = task_spec.get("task_id", False) # A task_id can be an int or a string. We need to be careful to handle both cases
        if not task_id:
            rospy.logwarn("task_id not found in task_spec")
            task_id = -1 # this should be an invalid task_id. We will not cross reference the task_id when we receive a cancel_current_task message. We will just cancel the current task regardless of the task_id
        state_machine = smach.StateMachine(outcomes=task_outcomes)
        with state_machine:
            rospy.logdebug("Adding states to smach")
            rospy.logdebug("Adding states from mission_spec to smach")
            for state_spec in task_spec["states"]:
                assert isinstance(state_spec, dict)
                state_spec: Dict = state_spec

                state_name = state_spec["name"]
                class_name = state_spec.get("class", None)
                rospy.logdebug(f"Add {state_name} to smach")

                if state_name in ["MISSION_PREPARATION", "MissionPreparation"]:
                    rospy.logdebug("Skipped MissionPreparation")
                    continue

                state_args = state_spec.get("args", {})
                state_args["task_id"] = task_id
                state_args = self.build_kwargs(state_args)
                state_args['name'] = state_name

                # make sure we initalize all task states with the task_outcomes
                # we want our states to recognize these outcomes so they initialize
                # with the extra functionality needed to handle these outcomes
                # so get the
                outcomes = set(state_args.get("outcomes", []))
                outcomes.update(task_outcomes)


                transition_spec = state_spec["transitions"]

                # We  want all these task outcomes to be in the args[outcomes] list
                state_outcomes = state_args.get("outcomes", set())
                state_outcomes.update(task_outcomes)

                state_args["outcomes"] = list(state_outcomes)
                # now we can be sure our state will be initialized with a keyword
                # argument `outcomes` that includes all the task outcomes
                smach_args = self.state_factory(state_name, transition_spec, state_args, class_name)

                # smach_args creates all the keyword args we need to add a state
                # to our smach machine.
                # but we need to update the `transitions` so that it doesn't
                # remap any of the task_outcomes. We want to make sure that the
                # all the task_outcomes are recognized as terminal outcomes
                # if the state machine sees the state reach one of these outcomes
                # the smach machine should exit with this outcome.
                # therefore it doesn't make sense for us to remap these
                # outcomes to some state in the state machine
                transitions_remap = smach_args["transitions"]
                for outcome in task_outcomes:
                    if outcome in transitions_remap:
                        # remove this
                        del transitions_remap[outcome]
                smach_args["transitions"] = transitions_remap
                smach.StateMachine.add(**smach_args)

                rospy.logdebug("Done with " + state_name)
            rospy.logdebug("Done adding states from mission_spec")


            rospy.logdebug("Finished adding states to smach")

        return state_machine, custom_outcomes

import dr_onboard_autonomy.airlease as airlease
from dr_onboard_autonomy.airlease import AirLeaserModel
from dr_onboard_autonomy.air_lease_service import AirLeaseService
from dr_onboard_autonomy.drone.mavros import ArdupilotCopterMavrosDrone, Px4CopterMavrosDrone
from dr_onboard_autonomy.mavlink import MavlinkNode, MavlinkSender
from dr_onboard_autonomy.message_senders import ReusableMessageSenders
from dr_onboard_autonomy.models import CopterDrone, GimbalAxes

from dr_onboard_autonomy.mqtt_client import MQTTClient

from dr_onboard_autonomy.briar_helpers import BriarLla
from dr_onboard_autonomy.state_factory import (
    init_state as _default_state_factory,
    StateSpec,
)

# This will register all states with the state_factory module. This is because
# accessing dr_onboard_autonomy.states will import every state
from dr_onboard_autonomy.states import AwaitMAVROS, AwaitFCU, ReadPosition, SetSystemClock
from dr_onboard_autonomy.states.components.VideoMetadata import open_csv_writer as open_video_metadata_file

StateFactoryFunction = Callable[[str, List, Dict, Optional[str]], StateSpec]
MQTTArg = Union[MQTTClient, str, None] # where the str is the hostname and the port is assumed to be 1883

from dr_onboard_autonomy.gimbal import GimbalGremsy, NewGimbalManager
from dr_onboard_autonomy.gimbal.geolocation import ThreeAxisGimbalQuatCalculator, TwoAxisGimbalQuatCalculator, FixedCameraQuatCalculator
from dr_onboard_autonomy.gimbal.geolocator import GeoLocator