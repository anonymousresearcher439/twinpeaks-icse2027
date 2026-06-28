# Notes about the Preflight Checks

This is an overview of the preflight checks that are run on the onboard pilot before arming occurs. The code for these checks is in the `Preflight` state located in the `states/Preflight.py` file.

---

First, there are the parameter checks, which ensure that certain test ratios are meant for estimator checks. Additionally, they provide a way for us to ensure that the connection to the flight controller is working and that the parameters are available. The parameters that are checked are:

- [`COM_ARM_EKF_HGT`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#COM_ARM_EKF_HGT)
- [`COM_ARM_EKF_VEL`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#COM_ARM_EKF_VEL)
- [`COM_ARM_EKF_POS`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#COM_ARM_EKF_POS)
- [`COM_ARM_EKF_YAW`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#COM_ARM_EKF_YAW)
- [`COM_ARM_IMU_ACC`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#COM_ARM_EKF_ACC)
- [`COM_ARM_IMU_GYR`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#COM_ARM_EKF_GYR)

The thresholds for these values are located in the [`parameters`](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/preflight/src/dr_onboard_autonomy/states/Preflight.py#L42) variable in the `Preflight` class and the values for these parameters are fetched every 5 seconds until all parameters pass the checks using the [`_get_param()`](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/preflight/src/dr_onboard_autonomy/mavros_layer.py#L103) method in the `MAVROSDrone` class located in the `mavros_layer.py` file. More detailed information about what these parameters control and what different errors may represent can be found on the PX4 website at [EKF Preflight Checks/Errors](https://docs.px4.io/v1.9.0/en/flying/pre_flight_checks.html#ekf-preflight-checkserrors). 

In addition to these parameters, a circular geofence is set and checked using the [`GF_MAX_HOR_DIST`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#GF_MAX_HOR_DIST) parameter. This provides a base geofence for the drone, and the behavior when the drone goes outside of the geofence boundary can be set using the [`GF_ACTION`](https://docs.px4.io/master/en/advanced_config/parameter_reference.html#GF_ACTION) parameter. 

---

Next, there are the estimator status check, which ensure that the GPS estimates for certain height and velocity measurement are providing accurate data. The estimators that are checked are:

- Horizontal velocity estimator
- Vertical velocity estimator
- Horizontal position estimator
- Vertical position (absolute) estimator
- Vertical position (above ground) estimator

These estimators are checked whenever a message is received on the `mavros/estimator_status` topic, which returns an [`EstimatorStatus`](http://docs.ros.org/en/api/mavros_msgs/html/msg/EstimatorStatus.html) message.

---

Next, certain sensors are checked, which ensure that necessary sensors are working. The sensors are all checked using the [MAV_SYS_STATUS_SENSOR](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR) bitmap. The specific sensors that are used are the same as used in QGroundControl ([found here](https://github.com/mavlink/qgroundcontrol/blob/master/src/FlightDisplay/PreFlightSensorsHealthCheck.qml#L21)) and are listed here:

- [`MAV_SYS_STATUS_SENSOR_3D_GYRO`](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR_3D_GYRO)
- [`MAV_SYS_STATUS_SENSOR_3D_ACCEL`](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR_3D_ACCEL)
- [`MAV_SYS_STATUS_SENSOR_3D_MAG`](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR_3D_MAG)
- [`MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE`](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)
- [`MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE`](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE)
- [`MAV_SYS_STATUS_SENSOR_GPS`](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR_GPS)
- [`MAV_SYS_STATUS_AHRS`](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_AHRS)
- [`MAV_SYS_STATUS_SENSOR_RC_RECEIVER`](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_SENSOR_RC_RECEIVER)
- [`MAV_SYS_STATUS_PREARM_CHECK`](https://mavlink.io/en/messages/common.html#MAV_SYS_STATUS_PREARM_CHECK)

While these are all used on QGroundControl, there have been some issues with certain sensors, mainly the `MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE` and `MAV_SYS_STATUS_PREARM_CHECK`, though further testing will be needed on these two. These sensors are checked whenever a message is received on the `/diagnostics` topic, which returns a [`DiagnosticArray`](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticArray.html) containing and array of [`DiagnosticStatus`](http://docs.ros.org/en/api/diagnostic_msgs/html/msg/DiagnosticStatus.html) messages.

---

Next, additional GPS checks are run to check the number of satellites and the GPS fix. The current threshold for satellites is 6, though this can be changed with the [`satellite_threshold`](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/preflight/src/dr_onboard_autonomy/states/Preflight.py#L79) variable in the `Preflight` class. Additionally the [GPS fix](https://mavlink.io/en/messages/common.html#GPS_FIX_TYPE) must be at least a 3D fix. Similar to the sensor checks, the GPS checks also occur whenever a message is received on the `/diagnostics` message.

---

The last checks that are run are ensuring that the drone is currently landed, not armed, and has a battery value of at least [`battery_threshold`](https://github.com/DroneResponse/DR-OnboardAutonomy/blob/preflight/src/dr_onboard_autonomy/states/Preflight.py#L76). 
