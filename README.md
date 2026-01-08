# Projects Incubator


## Project list

Legend
- 📝: NOT STARTED
- 🔎: ANALYZING
- ⏳: WORK IN PROGRESS
- 🕒: WAITING
- ✅: COMPLETED


### Robotics


#### VentuRobotics prototypes

| Status | Project                                                                                       | Description                                                    |
| ------ | --------------------------------------------------------------------------------------------- | -------------------------------------------------------------- |
| 📝      | [RPi Motor Control Protocol](rpi_motor_control_protocol/README.md)                            | Motor control protocols (PWM, OneShot, DShot)  implementations |
| ⏳      | [Model Predictive Control](https://github.com/Vince-94/model_predictive_control)              | Model Preditive Control implementations                        |
| 📝      | [Sensor Fusion with AI](sensor_fusion_ai/README.md)                                           | Sensor fusion with AI                                          |
| 📝      | [UGV Fleet Management](ugv_fleet_management/README.md)                                        | Fleet management for UGVs                                      |
| ⏳      | [Remote robotics teleop](https://github.com/Vince-94/robotics_remote_teleoperation)           | Remote teleoperation via web-socket                            |
| 📝      | [Flight Telemetry Analytics Platform](https://github.com/Vince-94/flight_telemetry_analytics) | Telemetry collector and analytics for UAVs                     |
| ✅      | [Robotics containerization](https://github.com/Vince-94/robotics_containerization)            | CLI that create ROS/micro-ROS configurable docker containers   |
| ⏳      | [Micro-ROS integration](https://github.com/Vince-94/micro_ros_integration)                    | CLI that helps to build and deploy micro-ROS applications      |
| 📝      | [Robotics cross-compilation](robotics_cross_compilation/README.md)                            | Cross-compilation tool utils for ROS2                          |
| 📝      | [Kalman Filter](kalman_filter/README.md)                                                      | Kalman Filter implementation                                   |
| 📝      | [Navigation algorithms](navigation_algorithm/README.md)                                       | Navigation algorithms implementation                           |
| 📝      | [Computer Vision Navigation](computer_vision_navigation/README.md)                            | Navigation using computer vision technicques                   |
| 📝      | [Real-time safety, watchdog, tracing & test harness]()                                        |                                                                |


#### Standalone

| Status | Project                                                          | Description |
| ------ | ---------------------------------------------------------------- | ----------- |
| 📝      | [Real Time Object Tracking](real_time_object_tracking/README.md) |             |
| 🕒      | [Autonomous Coilgun](autonomous_coilgun/README.md)               |             |




### Hacking

| Status | Project                                                           | Description |
| ------ | ----------------------------------------------------------------- | ----------- |
| ✅      | [RFID/NFC Item Tracking System](rfid_nfc_item_tracking/README.md) |             |
|        |                                                                   |             |


### Others

| Status | Project                                                                       | Description |
| ------ | ----------------------------------------------------------------------------- | ----------- |
| ✅      | [LED Blink](led_blink/README.md)                                              |             |
| ✅      | [Calendar alert telegram bot](https://github.com/Vince-94/calendar_alert_bot) |             |




## Cython for python scripts
1. Deps: `pip install cython setuptools wheel`
2. Place `setup_cython.py` at root level
3. `python3 setup_cython.py`
