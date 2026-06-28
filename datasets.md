# Datasets

[Back to README](README.md)

Version snapshots selected from five UAV-related software systems for longitudinal evaluation.
File counts and KLOC are restricted to the analysed subsystem.

---

### DROnboard (Java) — Closed Source, used with permission
Onboard autonomy subsystem for mission execution and vehicle coordination within the DroneResponse ecosystem.

| Ver. | Branch / Tag | Date | Files | KLOC | Key architectural change |
|---|---|---|---:|---:|---|
| V0 | `dev-briar` | 2022-07 | 80 | 9.9 | Briar framework integration |
| V1 | `dev-248-add-logging` | 2023-06 | 122 | 19.8 | Logging and MQTT additions |
| V2 | `dev-375-ardupilot-sim` | 2024-06 | 166 | 32.7 | ArduPilot MAVLink implementation |
| V3 | `dev-468` | 2024-12 | 186 | 41.4 | Home-position reset feature |
| V4 | `stable` | 2025-11 | 212 | 49.2 | Production-stable release |

→ [Source snapshots](datasets/dronboard/versions.md)

---

### [Dronology](<!-- TODO: add GitHub URL -->) (Java)
Research platform for ground control centralized coordination, monitoring, & management of UAV swarms.

| Ver. | Branch / Tag | Date | Files | KLOC | Key architectural change |
|---|---|---|---:|---:|---|
| V0 | `DATASET_V001` | 2018-05-11 | 261 | 17.6 | Baseline: core UAV coordination framework |
| V1 | `integration@cc927a2` | 2018-06-19 | 302 | 21.8 | Post-merge integration snapshot |
| V2 | `integration@902769b` | 2018-07-12 | 357 | 25.9 | Area-mapping and mission-planning extensions |
| V3 | `ICSE_2019_DATA_V1` | 2018-11-13 | 392 | 26.9 | ICSE 2019 published dataset (V1) |
| V4 | `integration` | 2019-02-12 | 405 | 27.7 | Latest stable integration branch |

→ [Source snapshots](datasets/dronology/versions.md)

---

### [PX4 Arming](https://github.com/PX4/PX4-Autopilot) (C++)
Safety-critical subsystem for validating vehicle readiness and enforcing pre-flight arming constraints.

| Ver. | Branch / Tag | Date | Files | KLOC | Key architectural change |
|---|---|---|---:|---:|---|
| V0 | `v1.14.0` | 2023-08-10 | 83 | 14.2 | `HealthAndArmingChecks` framework stabilised |
| V1 | `v1.15.0` | 2024-08-23 | 85 | 14.8 | `ArmStateMachine` simplified; race-condition fix |
| V2 | `v1.16.0` | 2025-08-05 | 91 | 15.4 | Add arming checks; failsafe msg improvements |
| V3 | `v1.17.0` | 2026-01-16 | 93 | 15.7 | Mode-management updates; failsafe refinements |
| V4 | `HEAD` | 2026-06-19 | 104 | 17.0 | Failure-detect → YAML; new arming checks |

→ [Source snapshots](datasets/px4/versions.md)

---

### [ArduPilot](https://github.com/ArduPilot/ardupilot) (C++)
ArduPilot multirotor flight controller subsystem for autonomy, navigation, & mission execution.

| Ver. | Branch / Tag | Date | Files | KLOC | Key architectural change |
|---|---|---|---:|---:|---|
| V0 | `ArduCopter-2.5` | 2012-03-17 | 41 | 16.1 | Arduino monolith: single sketch w. direct includes |
| V1 | `ArduCopter-2.9.1` | 2013-02-02 | 66 | 22.0 | `AP_Motors` → new library; `AP_AHRS` added |
| V2 | `ArduCopter-3.0-rc4` | 2013-06-02 | 78 | 23.5 | HW abstract. Nav & Fence extract. INU added. |
| V3 | `ArduCopter-3.1.0-rc7` | 2013-11-22 | 86 | 25.6 | Battery mon, Mapper, new modes. |
| V4 | `Copter-3.3.1` | 2015-10-26 | 139 | 45.2 | `.pde` → `.cpp`; Copter class, EKF, Att-Ctrl |

→ [Source snapshots](datasets/ardupilot/versions.md)

---

### [AeroStack2](https://github.com/aerostack2/aerostack2) (C++)
ROS2-based aerial robotics framework for autonomy and multi-UAV coordination.

| Ver. | Branch / Tag | Date | Files | KLOC | Key architectural change |
|---|---|---|---:|---:|---|
| V0 | `v0.2.1` | 2022-12-19 | 127 | 18.5 | Monolithic ctrl; platform drivers bundled |
| V1 | `1.0.0` | 2023-03-18 | 178 | 24.2 | `ControllerBase` + pluginlib; BT orch. layer |
| V2 | `1.0.5` | 2023-11-08 | 180 | 25.2 | Plugin lib. compilation stabilised; DJI OSDK exp. |
| V3 | `1.1.0` | 2024-08-08 | 190 | 30.9 | Platform drivers ext. → repos; A* added |
| V4 | `1.1.3` | 2025-07-23 | 216 | 35.7 | Swarm flock.; Voronoi plan.; multi-UAV coord |

→ [Source snapshots](datasets/aerostack2/versions.md)
