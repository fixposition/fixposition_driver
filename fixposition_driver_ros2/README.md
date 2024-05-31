# Fixposition Driver ROS2

[ROS](https://www.ros.org/) (both ROS1 and ROS2) Driver for [Fixposition Vision-RTK 2](https://www.fixposition.com/product)

## Dependencies

-  [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), tested with version [3.3.7](https://gitlab.com/libeigen/eigen/-/releases/3.3.7)
-  [Boost](https://www.boost.org/), tested with version [1.65.0](https://www.boost.org/users/history/version_1_65_0.html)
-  [CMake](https://cmake.org/)
-  [Transforms](https://wiki.ros.org/tf)
-  [Colcon](https://colcon.readthedocs.io/en/released/)

-  **[fixposition_gnss_tf](https://github.com/fixposition/fixposition_gnss_tf)**: Fixposition GNSS Transformation Lib


This driver operates as a ROS node, connecting to either a TCP or serial stream of Fixposition Vision-RTK output data, see [Fixposition ASCII messages](#fixposition-ascii-messages) and the **Integration Manual**.

## Installing dependencies on Ubuntu system

```
 sudo apt update
 sudo apt install -y build-essential cmake
 sudo apt install -y libeigen3-dev
 sudo apt install -y libboost-date-time-dev 
```


## Installation

To install the node, extract / clone the code and `fixposition_gnss_tf` to your catkin workspace's `src` folder:

```bash
# The folder structure should look like this
fp_public_ws
├── src
│   ├── fixposition_driver
│   │   ├── fixposition_driver_lib
│   │   ├── fixposition_driver_ros1 # will be ignored by colcon when building for ROS2
│   │   ├── fixposition_driver_ros2
│   ├── fixposition_gnss_tf
```
make sure you have sourced the setup.bash from ros:

`/opt/ros/{ROS_DISTRO}/setup.bash`, for example

```
source /opt/ros/foxy/setup.bash
```

and build it with:

`colcon build --packages-up-to fixposition_driver_ros2`

This will build the ROS2 driver node and all its dependencies.


Then source your environment after the build:

`source install/setup.bash`

## Launch the Driver

-  To launch the node in serial mode, run:

   `ros2 launch fixposition_driver_ros2 serial.launch`

-  In TCP mode (Wi-Fi):

   `ros2 launch fixposition_driver_ros2 tcp.launch`

-  In TCP mode (Ethernet):

   `ros2 launch fixposition_driver_ros2 tcp.launch`

To change the settings of TCP (IP, Port) or Serial (Baudrate, Port) connections, check the `launch/tcp.yaml` and `launch/serial.yaml` files.

## Check the messages
- Check rostopics:

  `ros2 topic list`


   there should be topics under the `/fixposition` namespace, for example:
   ```bash
   /fixposition/corrimu
   /fixposition/navsatfix
   /fixposition/odometry
   /fixposition/odomsh
   /fixposition/odometry_enu
   /fixposition/poiimu
   /fixposition/rawimu
   /fixposition/vrtk
   ```

- Check published message in the topic, for example the VRTK message:
   `ros2 topic echo /fixposition/vrtk`


## Output of the driver

### Messages and TF tree

The output is published on the following:

#### Vision-RTK2 Fusion

-   From **FP_A-ODOMETRY**, at the configured frequency (default 10Hz, output generator -> Fusion frequency):

    -   Messages

    | Topic                       | Message Type              | Frequency                      | Description                                                                                                                                                   |
    | --------------------------- | ------------------------- | ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
    | `/fixposition/odometry`     | `nav_msgs/Odometry`       | as configured on web-interface | Position, Orientation from ECEF to FP_POI, Velocity and Angular Velocity in FP_POI                                                                            |
    | `/fixposition/odomsh`       | `nav_msgs/Odometry`       | as configured on web-interface | Position, Orientation from ECEF to FP_POISH, Velocity and Angular Velocity in FP_POI. Based on smooth odometry output.                                        |
    | `/fixposition/navsatfix`    | `sensor_msgs/NavSatFix`   | as configured on web-interface | Position from LLH (Latitude, Longitude and Height) to FP_POI. Based on WGS-84 ellipsoid.                                                                      |
    | `/fixposition/odometry_enu` | `nav_msgs/Odometry`       | as configured on web-interface | Position, Orientation from ENU0 to FP_POI, Velocity and Angular Velocity in FP_POI                                                                            |
    | `/fixposition/vrtk`         | `fixposition_driver/VRTK` | as configured on web-interface | Custom Message containing same Odometry information as well as status flags                                                                                   |
    | `/fixposition/poiimu`       | `sensor_msgs/Imu`         | as configured on web-interface | Bias Corrected acceleration and rotation rate in FP_POI                                                                                                       |
    | `/fixposition/ypr`          | `geometry_msgs/Vector3`   | as configured on web-interface | x = Yaw, y = Pitch, z = Roll in radian. Euler angles representation of rotation between ENU and FP_POI. Only available after fusion initialization.           |    

#### Vision-RTK2 GNSS Antenna Positions

**If GNSS Antenna positions are needed, please enable this on the sensor's configuration interface.**

-   From **NOV_B-BESTGNSSPOS_GNSS[1,2]**, at the configured frequency, GNSS1 and GNSS2 raw antenna positions (default 5Hz):

    | Topic                | Message Type            | Frequency                      | Description                    |
    | -------------------- | ----------------------- | ------------------------------ | ------------------------------ |
    | `/fixposition/gnss1` | `sensor_msgs/NavSatFix` | as configured on web-interface | Latitude, Longitude and Height |
    | `/fixposition/gnss2` | `sensor_msgs/NavSatFix` | as configured on web-interface | Latitude, Longitude and Height |

-   From **NMEA-GP-GGA_GNSS**, **NMEA-GP-RMC_GNSS**, and **NMEA-GP-ZDA_GNSS**, at the configured frequency (default 5Hz): The Vision-RTK2 can also output an average GNSS-based LLH position (i.e., **only GNSS, not Fusion**) and heading estimate based on speed over ground (**SOG**) and course over ground (**COG**) - (i.e., **the platform must be moving for it to be accurate**) by utilizing several NMEA messages, which serves as an auxiliary output until Fusion is fully initialized. This message's output frequency equals the lowest frequency of any of these three message types. **Note: This output should only be used until Fusion is fully initialized.** Warning: This topic will only be populated when the sampling rate of all required NMEA messages is 1 (i.e., the default output frequency of the messages).

    | Topic                | Message Type            | Frequency                      | Description                    |
    | -------------------- | ----------------------- | ------------------------------ | ------------------------------ |
    | `/fixposition/nmea` | `fixposition_driver/NMEA` | as configured on web-interface | Latitude, Longitude and Height |

#### Vision-RTK2 IMU data

-   From **FP_A-RAWIMU**, at 200Hz:

    | Topic                 | Message Type      | Frequency | Description                                                                               |
    | --------------------- | ----------------- | --------- | ----------------------------------------------------------------------------------------- |
    | `/fixposition/rawimu` | `sensor_msgs/Imu` | 200Hz     | Raw (without bias correction) IMU acceleration and angular velocity data in FP_VRTK frame |

-   From **FP_A-CORRIMU**, at 200Hz:

    | Topic                  | Message Type      | Frequency | Description                                                                |
    | ---------------------- | ----------------- | --------- | -------------------------------------------------------------------------- |
    | `/fixposition/corrimu` | `sensor_msgs/Imu` | 200Hz     | Bias Corrected IMU acceleration and angular velocity data in FP_VRTK frame |

-   From **FP_A-TF_POIIMUH**, at 200Hz:

    | Topic                    | Message Type            | Frequency                      | Description                    |
    | ------------------------ | ----------------------- | ------------------------------ | ------------------------------ |
    | `/fixposition/imu_ypr`   | `geometry_msgs/Vector3` | 200Hz                          | x = 0.0, y = Pitch, z = Roll in radian. Euler angles representation of rotation between a local horizontal frame and P_POI. Rough estimation using IMU alone. |

#### Transforms

-   TFs:
    | Frames              | Topic        | Message needed to be selected on the web-interface | Frequency                          |
    | ------------------- | ------------ | -------------------------------------------------- | ---------------------------------- |
    | `FP_ECEF-->FP_POI`  | `/tf`        | `ODOMETRY`                                         | as configured on the web-interface |
    | `FP_ECEF-->FP_MAP`  | `/tf_static` | `ODOMETRY`                                         | 1Hz                                |
    | `FP_ECEF-->FP_ENU`  | `/tf`        | `ODOMENU`                                          | as configured on the web-interface |
    | `FP_ECEF-->FP_ENU0` | `/tf_static` | `ODOMENU`                                          | 1Hz                                |
    | `FP_POI-->FP_POISH` | `/tf`        | `ODOMSH`                                           | as configured on the web-interface |
    | `FP_POI-->FP_IMUH`  | `/tf`        | `TF_POIIMUH`                                       | 200Hz                              |
    | `FP_POI-->FP_VRTK`  | `/tf_static` | `TF_POI_VRTK`                                      | 1Hz                                |
    | `FP_VRTK-->FP_CAM`  | `/tf_static` | `TF_VRTK_CAM`                                      | 1Hz                                |
    

-   ROS TF Tree:

    ```mermaid
    graph TD;
    FP_ECEF-->FP_POI
    FP_POI-->FP_VRTK-->FP_CAM
    FP_POI-->FP_IMUH
    FP_POI-->FP_POISH
    FP_ECEF-->FP_ENU0
    ```

_Please note that the corresponding messages also has to be selected on the Fixposition Vision-RTK2's configuration interface._

### Explaination of frame ids

| Frame ID     | Explaination                                                                                                                                                   |
| ------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **FP_ECEF**  | Earth-Center-Earth-Fixed frame.                                                                                                                                |
| **FP_POI**   | Point-Of-Interest, configured from Vision-RTK2's web-interface with respect to the FP_VRTK frame. By default it is the same as FP_VRTK.                        |
| **FP_VRTK**  | The coordinate frame on the Vision-RTK2's housing on the Fixposition-Logo "X".                                                                                 |
| **FP_CAM**   | The camera coordinate frame of the Vision-RTK2.                                                                                                                |
| **FP_POISH** | Point-Of-Interest of the Smooth Odometry frame, configured from Vision-RTK2's web-interface with respect to the FP_VRTK frame.                                 |
| **FP_IMUH**  | A local horizontal frame with the origin at the same location as FP_POI. This frame is a rough estimate determined by the IMU alone.                           |
| **FP_ENU0**  | The **global fixed** East-North-Up coordinate frame with the origin at the configured ENU0 position on the web-interface. Also used as the MAP frame for Rviz. |

# Input Wheelspeed through the driver

The fp_ros_driver supports inputting a Speed msg (`msg/Speed.msg`) through the `/fixposition/speed` topic. The Speed msg is defined as a vector of WheelSensor msgs (`msg/WheelSensor.msg`). This message in turn, is intended to be a simplified version of the FP_B-MEASUREMENTS, containing three integers (vx, vy, and vz), three booleans (validity of the three velocity integers), and a string indicating the sensor from which the measurement originates. The integer velocity values should be in [mm/s], to have enough precision when being converted into an integer format.

Internally, upon arriving to the ros driver, wheelspeed measurements are converted into a full FP_B-MEASUREMENTS message, and sent via the TCP or serial interface to the Vision-RTK2, where they will be further processed. For more details regarding the definition FP_B-MEASUREMENTS message, please refer to [the following page](https://docs.fixposition.com/fd/fp_b-measurements), or to the VRTK2 integration manual.

# Fixposition Odometry Converter

This is an extra node is provided to help with the integration of the wheel odometry on your vehicle. For details, see the subfolder [fixposition_odometry_converter_ros2](fixposition_odometry_converter_ros2/README.md) (ROS 2).

## Fixposition ASCII messages

For more information about the ASCII messages parsed from the Vision-RTK 2, please refer to [Fixposition Docs](https://docs.fixposition.com/fd/i-o-messages).

## Code Documentation

Run `doxygen Doxyfile` to generate Doxygen code documentation.

# License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
