# ArduPilot — Version Snapshots

[Back to datasets](../../datasets.md)

ArduPilot multirotor flight controller subsystem for autonomy, navigation, & mission execution.

| Ver. | Branch / Tag | Date | Files | KLOC | Key architectural change |
|---|---|---|---:|---:|---|
| [V0](v0/) | `ArduCopter-2.5` | 2012-03-17 | 41 | 16.1 | Arduino monolith: single sketch w. direct includes |
| [V1](v1/) | `ArduCopter-2.9.1` | 2013-02-02 | 66 | 22.0 | AP_Motors → new library; AP_AHRS added |
| [V2](v2/) | `ArduCopter-3.0-rc4` | 2013-06-02 | 78 | 23.5 | HW abstract. Nav & Fence extract. INU added. |
| [V3](v3/) | `ArduCopter-3.1.0-rc7` | 2013-11-22 | 86 | 25.6 | Battery mon, Mapper, new modes. |
| [V4](v4/) | `Copter-3.3.1` | 2015-10-26 | 139 | 45.2 | .pde → .cpp; Copter class, EKF, Att-Ctrl |
