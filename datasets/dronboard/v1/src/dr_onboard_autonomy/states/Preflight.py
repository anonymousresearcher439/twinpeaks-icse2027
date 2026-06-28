import math

import rospy
from pymavlink import mavutil

from dr_onboard_autonomy.message_senders import RepeatTimer

from .BaseState import BaseState


class Preflight(BaseState):
    def __init__(self, **kwargs):
        kwargs["outcomes"] = ["succeeded_preflight", "error"]
        super().__init__(**kwargs)
        msg_sender_names = [
            "battery",
            "extended_state",
            "position",
            "state",
            "diagnostics",
            "estimator_status",
        ]

        # Create repeat timer and for checking parameters
        self.param_time_limit = 5
        self.param_check_count = 0
        self.param_check_status = False

        self.param_check_timer = RepeatTimer("param_check_timer", self.param_time_limit)
        self.message_senders.add(self.param_check_timer)

        # Create repeat timer to exit preflight state with error after certain amount of time
        self.failure_time_limit = 120

        self.failure_timer = RepeatTimer("failure_timer", self.failure_time_limit)
        self.message_senders.add(self.failure_timer)

        for sender_name in msg_sender_names:
            message_sender = self.reusable_message_senders.find(sender_name)
            self.message_senders.add(message_sender)

        self.handlers.add_handler("param_check_timer", self.on_param_timer_message)
        self.handlers.add_handler("failure_timer", self.on_failure_timer_message)

        for message_type in msg_sender_names:
            self.handlers.add_handler(message_type, self.on_ros_message)

        # List of preflight parameters with thresholds
        self.parameters = {
            "COM_ARM_EKF_HGT": {"threshold": 1.0, "available": None, "previous": None},
            "COM_ARM_EKF_VEL": {"threshold": 1.0, "available": None, "previous": None},
            "COM_ARM_EKF_POS": {"threshold": 1.0, "available": None, "previous": None},
            "COM_ARM_EKF_YAW": {"threshold": 1.0, "available": None, "previous": None},
            "COM_ARM_IMU_ACC": {"threshold": 1.0, "available": None, "previous": None},
            "COM_ARM_IMU_GYR": {"threshold": 0.3, "available": None, "previous": None},
        }

        # List of
        self.sensors = {
            "Gyrometer": {
                "value": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO,
                "previous": None,
            },
            "Accelerometer": {
                "value": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL,
                "previous": None,
            },
            "Magnetometer": {
                "value": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG,
                "previous": None,
            },
            "Absolute Pressure": {
                "value": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE,
                "previous": None,
            },
            # "Differential Pressure": {"value": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, "previous": None}, # Used on QGroundControl, but does not work with simulator
            "GPS": {
                "value": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS,
                "previous": None,
            },
            "AHRS": {"value": mavutil.mavlink.MAV_SYS_STATUS_AHRS, "previous": None},
            # "RC Receiver": {"value": mavutil.mavlink.MAV_SYS_STATUS_SENSOR_RC_RECEIVER, "previous": None}, # Not needed for simulated drones
            "Prearm Check": {
                "value": mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK,
                "previous": None,
            },  # Should pass when drone is ready to fly
        }

        self.estimators = {
            "velocity_horiz_status_flag": {
                "name": "Horizontal velocity estimator",
                "previous": None,
            },
            "velocity_vert_status_flag": {
                "name": "Vertical velocity estimator",
                "previous": None,
            },
            "pos_horiz_abs_status_flag": {
                "name": "Horizontal position estimator",
                "previous": None,
            },
            "pos_vert_abs_status_flag": {
                "name": "Vertical position (absolute) estimator",
                "previous": None,
            },
            "pos_vert_agl_status_flag": {
                "name": "Vertical position (above ground) estimator",
                "previous": None,
            },
        }

        # TODO ask about this value
        self.battery_threshold = 0.2

        # Require a 6 satellites and 3D fix to pass preflight checks
        self.satellite_threshold = 6
        self.gps_fix_threshold = 3
        self.last_gps_fix = 0

        # Set radius for circular geofence
        self.geofence_radius = 500.0
        self.geofence_previous = None
        # self.set_geofence()

    def set_geofence(self):
        # Set circular geofence at specified radius
        fence_result = self.drone._set_param(
            "GF_MAX_HOR_DIST", real_value=self.geofence_radius
        )
        if fence_result:
            msg = f"Geofence has been set to {self.geofence_radius} meters"
            self.mqtt_client.arming_status_update(msg, "success")
            rospy.loginfo(msg)
        else:
            msg = "Unable to set the geofence"
            self.mqtt_client.arming_status_update(msg, "error")
            rospy.logerr(msg)

    # def set_disarm_preflight(self, disarm_time: float=120.0):
    #     """Sets the threhold for disarming the drone after arming without a subsequent transition
    #     Disarm time in seconds with negative values setting indefinite limit
    #     """
    #     disarm_result = self.drone._set_param(
    #         "COM_DISARM_PRFLT", real_value=120.0
    #     )
    #     if disarm_result:
    #         msg = f"Disarm with no transition after arming set to {disarm_time} seconds"
    #         self.mqtt_client.arming_status_update(msg, "success")
    #         rospy.loginfo(msg)
    #     else:
    #         msg = "Unable to set disarm preflight threshold"
    #         self.mqtt_client.arming_status_update(msg, "error")
    #         rospy.logerr(msg)

    def on_param_timer_message(self, _):
        param_status = True

        # Check parameters
        for param, info in self.parameters.items():
            param_value = self.drone._get_param(param)
            # If parameter succeeds compare to previous values and send message on status change
            if param_value.success:
                info["available"] = True
                if param_value.value.real != info["previous"]:
                    info["previous"] = param_value.value.real
                    if param_value.value.real > info["threshold"]:
                        msg = "Pre-Arm Error: Parameter {} is above threshold".format(
                            param
                        )
                        self.mqtt_client.arming_status_update(msg, "error")
                        rospy.logerr(msg)
                        param_status = False
                    else:
                        msg = "Pre-Arm: Parameter {} passed pre-arm check".format(param)
                        self.mqtt_client.arming_status_update(msg, "success")
                        rospy.loginfo(msg)
            # If the parameter has not been checked or was previously available, send error to DR
            elif info["available"] is not False:
                info["available"] = False
                msg = "Pre-Arm Error: Unable to get parameter {}".format(param)
                self.mqtt_client.arming_status_update(msg, "error")
                rospy.logerr(msg)
                param_status = False

        # Check that the geofence is set, only send message on status change
        geofence = self.drone._get_param("GF_MAX_HOR_DIST")
        if geofence.success and geofence.value.real != self.geofence_previous:
            self.geofence_previous = geofence.value.real
            if geofence.value.real == self.geofence_radius:
                self.drone.update_data("geofence_status", True)
                msg = "Pre-Arm: Geofence is set correctly"
                self.mqtt_client.arming_status_update(msg, "success")
                rospy.loginfo(msg)
            else:
                self.drone.update_data("geofence_status", False)
                msg = "Pre-Arm Error: Geofence is not set correctly"
                self.mqtt_client.arming_status_update(msg, "error")
                rospy.logerr(msg)
                self.set_geofence()
                param_status = False

        # set preflight disarm threshold
        # self.set_disarm_preflight()

        # Stop the RepeatTimer when the parameter checks pass
        if param_status:
            self.param_check_status = True
            self.param_check_timer.stop()

    def on_failure_timer_message(self, message):
        msg = "Unable to pass pre-flight state after 120 seconds. Exiting..."
        self.mqtt_client.arming_status_update(msg, "error")
        rospy.logerr(msg)
        return "error"

    def on_ros_message(self, message):
        # Don't run pre-flight when all data is not available
        is_data_available = all(
            [
                "battery" in self.message_data,
                "extended_state" in self.message_data,
                "position" in self.message_data,
                "state" in self.message_data,
                "diagnostics" in self.message_data,
                "estimator_status" in self.message_data,
            ]
        )

        if not is_data_available:
            return

        data = self.message_data
        is_ready = [
            data["extended_state"].landed_state
            == 1,  # we are on the ground when LANDED_STATE_ON_GROUND=1
            data["battery"].percentage >= self.battery_threshold
            or math.isnan(data["battery"].percentage),  # we have enough juice
            data["state"].armed != True,
            self.param_check_status,  # Ensure that parameters have passed checks
            self.check_gps(data["diagnostics"].status[1]),
            self.check_sensors(
                data["diagnostics"].status[3]
            ),  # sensor health bitmaps pass checks
            self.check_estimator_status(data["estimator_status"]),
        ]

        if all(is_ready):
            return "succeeded_preflight"

    def check_gps(self, data):
        # Ensure correct message was grabbed
        rospy.logdebug(f"check_gps data.name = {data.name}")
        if "mavros: GPS" in data.name:
            # Get satellite values and GPS fix type
            satellites_visible = int(data.values[0].value)
            gps_fix = int(data.values[1].value)

            # Check if visible satellites and fix type pass thresholds
            gps_healthy = (
                satellites_visible >= self.satellite_threshold
                and gps_fix >= self.gps_fix_threshold
            )

            # Output an error message if GPS fix has changed
            if gps_fix != self.last_gps_fix:
                # Get information about current fix from MAVLink
                gps_fix_info = mavutil.mavlink.enums["GPS_FIX_TYPE"][gps_fix]
                msg = "GPS Info: fix: {} -- {} | satellites: {}".format(
                    gps_fix_info.name, gps_fix_info.description, satellites_visible
                )
                if gps_healthy:
                    self.mqtt_client.arming_status_update(msg, "success")
                    rospy.loginfo(msg)
                else:
                    self.mqtt_client.arming_status_update(msg, "error")
                    rospy.logerr(msg)

                self.last_gps_fix = gps_fix

            return gps_healthy
        else:
            return False

    def check_sensors(self, data):
        rospy.logdebug(f"check_sensors data.name = {data.name}")
        # Ensure correct message was grabbed
        if "mavros: System" in data.name:
            all_healthy = True

            # Convert hex values of sensor health bitmaps to integers
            health_bitmap = int(data.values[2].value, 16)

            for name, bits in self.sensors.items():
                healthy = (
                    True
                    if ((health_bitmap & bits["value"]) == bits["value"])
                    else False
                )
                if not healthy:
                    all_healthy = False

                if bits["previous"] != healthy:
                    bits["previous"] = healthy
                    if healthy:
                        msg = "Pre-Arm: {} is calibrated/set-up properly".format(name)
                        self.mqtt_client.arming_status_update(msg, "success")
                        rospy.loginfo(msg)
                    else:
                        msg = "Pre-Arm Error: {} not calibrated/set-up properly".format(
                            name
                        )
                        self.mqtt_client.arming_status_update(msg, "error")
                        rospy.logerr(msg)

            return all_healthy

        else:
            return False

    def check_estimator_status(self, data):
        all_healthy = True

        for estimator, info in self.estimators.items():
            # Get status of estimator from the estimator_status message
            status = getattr(data, estimator)
            if not status:
                all_healthy = False

            if info["previous"] != status:
                info["previous"] = status
                if status:
                    msg = "Estimator success: {}".format(info["name"])
                    self.mqtt_client.arming_status_update(msg, "success")
                    rospy.loginfo(msg)
                else:
                    msg = "Estimator error: {}".format(info["name"])
                    self.mqtt_client.arming_status_update(msg, "error")
                    rospy.logerr(msg)

        return all_healthy
