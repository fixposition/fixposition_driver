# Changelog

_For questions about compatibility, please contact Fixpositions Support support@fixposition.com_

## [6.1.3](https://github.com/fixposition/fixposition_driver/releases/tag/6.1.3)

    - Improve README

## [6.1.2](https://github.com/fixposition/fixposition_driver/releases/tag/6.1.2)

    - Bugfix gnss message timestamps

## [6.1.1](https://github.com/fixposition/fixposition_driver/releases/tag/6.1.1)

    -   Bugfix for serial connection
    -   Add frame_id for imu messages

## [6.1.0](https://github.com/fixposition/fixposition_driver/releases/tag/6.1.0)

    -   Add feature to publish GNSS Antenna positions even if Fusion is not initialized. NOV_B-BESTGNSSPOS is used and sensor_msgs::NavSatFix is published. See more details in [Vision-RTK2 GNSS Antenna Positions](#Vision-RTK2-GNSS-Antenna-Positions).

## [6.0.3](https://github.com/fixposition/fixposition_driver/releases/tag/6.0.3)

    -   Fix Frame ID of FP_POI in LLH converter
    -   Bugfix of sending empty TFs
    -   Fix wrong gnss status
    -   Remove 'WsCallback' warning
    -   Add ci file for ROS

## [6.0.2](https://github.com/fixposition/fixposition_driver/releases/tag/6.0.2)

    -   Add missing depencency of `tf2_eigen` in `fixposition_driver_ros2/CMakeList.txt`

## [6.0.1](https://github.com/fixposition/fixposition_driver/releases/tag/6.0.1)

    -   Adapted to be compatible with updated Fixposition message definitions
    -   **Compatible with Vision-RTK 2 software released after 09.03.2023**

## [5.0.0](https://github.com/fixposition/fixposition_driver/releases/tag/5.0.0)

    -   Support both ROS1 and ROS2
    -   Diverse bugfixes in code and documentation
    -   **This is the last version compatible with Vision-RTK 2 software released before 17.01.2023**

## [4.4.0](https://github.com/fixposition/fixposition_driver/releases/tag/4.4.0)

    -   Pitch-Roll estimation from IMU data parsed from Vision-RTK 2, output available before fusion initialization.
    -   OdometryConverter imeplemented as an example for wheelspeed data integration.

## [4.3.0](https://github.com/fixposition/fixposition_driver/releases/tag/4.3.0)

    -   Automatic reconnect after connection is lost
    -   Adaption to latest Vision-RTK 2 software changes

## [4.2.0](https://github.com/fixposition/fixposition_driver/releases/tag/4.2.0)

    -   Euler Angle Yaw-Pitch-Roll in local ENU frame
    -   Odometry in fixed ENU0 frame

## [4.0.1](https://github.com/fixposition/fixposition_driver/releases/tag/4.0.1)

    -   First public release
    -   Code cleanup
