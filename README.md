# Fixposition ROS Driver

-   [ROS1 melodic / noetic ![](./../../actions/workflows/build_test_ros.yml/badge.svg)](./../../actions/workflows/build_test_ros.yml)
-   [ROS2 foxy / humble ![](./../../actions/workflows/build_test_ros2.yml/badge.svg)](./../../actions/workflows/build_test_ros2.yml)

[ROS](https://www.ros.org/) (both ROS1 and ROS2) Driver for [Fixposition Vision-RTK 2](https://www.fixposition.com/product).

The driver is designed to listen on a TCP or Serial port for the [_Fixposition ASCII Messages_](#fixposition-ascii-messages), and then publish them as corresponding ROS messages. At the same time, the driver can also subscribe to a speed input message, which will be sent back to the Vision-RTK 2 sensor and provide an external speed input.

-   For the output ROS messages, see [Output of the driver](#output-of-the-driver)
-   For the input ROS messages for speed input, see [Input Wheelspeed through the driver](#input-wheelspeed-through-the-driver)

## How to use it

The code is split in the following 3 parts:

-   `fixposition_driver_lib`: common CMake library to parse [_Fixposition ASCII Messages_](#fixposition-ascii-messages). For more details and build instructions, see [here](fixposition_driver_lib/README.md).
-   `fixposition_driver_ros1`: ROS1 driver node to subscribe and publish in the ROS1 framework. For more details and build instructions, see [here](fixposition_driver_ros1/README.md).
-   `fixposition_driver_ros2`: ROS2 driver node to subscribe and publish in the ROS2 framework. For more details and build instructions, see [here](fixposition_driver_ros2/README.md).

### ROS 1

For more details and build instructions, see [here](fixposition_driver_ros1/README.md).

#### Installation

To install the node, extract / clone the code and `fixposition_gnss_tf` to your catkin workspace's `src` folder:

```bash
# The folder structure should look like this
fp_public_ws
├── src
│   ├── fixposition_driver
│   │   ├── fixposition_driver_lib
│   │   ├── fixposition_driver_ros1
│   │   ├── fixposition_driver_ros2 # will be ignore by catkin
│   ├── fixposition_gnss_tf
```

**Add a file named 'CATKIN_IGNORE' to `fixposition_driver_ros2` folder.**

make sure you have sourced the setup.bash from ros:

`/opt/ros/{ROS_DISTRO}/setup.bash`, for example

```
source /opt/ros/melodic/setup.bash`
```

and build it with:

`catkin build fixposition_driver_ros1`

This will build the ROS1 driver node and all its dependencies.

Then source your development environment:

`source devel/setup.bash`

#### Launch the Driver

-   To launch the node in serial mode, run:

    `roslaunch fixposition_driver_ros1 serial.launch`

-   In TCP mode (Wi-Fi):

    `roslaunch fixposition_driver_ros1 tcp.launch`

-   In TCP mode (Ethernet):

    `roslaunch fixposition_driver_ros1 tcp.launch`

To change the settings of TCP (IP, Port) or Serial (Baudrate, Port) connections, check the `launch/tcp.yaml` and `launch/serial.yaml` files.

### ROS 2

For more details and build instructions, see [here](fixposition_driver_ros2/README.md).

#### Installation

To install the node, extract / clone the code and `fixposition_gnss_tf` to your catkin workspace's `src` folder:

```bash
# The folder structure should look like this
fp_public_ws
├── src
│   ├── fixposition_driver
│   │   ├── fixposition_driver_lib
│   │   ├── fixposition_driver_ros1 # will be ignore by colcon when building for ROS2
│   │   ├── fixposition_driver_ros2
│   ├── fixposition_gnss_tf
```

**Add a file named 'COLCON_IGNORE' to `fixposition_driver_ros1` folder.**

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

#### Launch the Driver

-   To launch the node in serial mode, run:

    `ros2 launch fixposition_driver_ros2 serial.launch`

-   In TCP mode (Wi-Fi):

    `ros2 launch fixposition_driver_ros2 tcp.launch`

-   In TCP mode (Ethernet):

    `ros2 launch fixposition_driver_ros2 tcp.launch`

**To change the settings of TCP (IP, Port) or Serial (Baudrate, Port) connections, check the `launch/tcp.yaml` and `launch/serial.yaml` files and read this note below.**

> [!NOTE]
> ROS2, unlike ROS1, by default uses a `install` directory in the workspace. So when you do `ros2 launch xxx`, the configuration and launch files are taken from the `install` and not directly from the `src` directory.
> 
> If you want to modify the parameters in the YAML files. You can:
>   - Modify the YAML file in the `src` directory and then re-run `colcon build --packages-up-to fixposition_driver_ros2` to update them into the `install` directory.
>
>       or 
>   - Modify the YAML file in `install`. However, the next time you do `colcon build` they will be overriden by the files in `src`.


## Output of the driver

### Messages and TF tree

The output is published on the following:

#### Vision-RTK2 Fusion

-   From **FP_A-ODOMETRY**, at the configured frequency (default 10Hz, output generator -> Fusion frequency):

    -   Messages

    | Topic                       | Message Type              | Frequency                      | Description                                                                                                                                                   |
    | --------------------------- | ------------------------- | ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
    | `/fixposition/odometry`     | `nav_msgs/Odometry`       | as configured on web-interface | Position, Orientation from ECEF to FP_POI, Velocity and Angular Velocity in FP_POI                                                                            |
    | `/fixposition/odomsh`       | `nav_msgs/Odometry`       | as configured on web-interface | Position, Orientation from ECEF to FP_POI, Velocity and Angular Velocity in FP_POI. Based on smooth odometry output.                                          |
    | `/fixposition/odometry_enu` | `nav_msgs/Odometry`       | as configured on web-interface | Position, Orientation from ENU0 to FP_POI, Velocity and Angular Velocity in FP_POI                                                                            |
    | `/fixposition/vrtk`         | `fixposition_driver/VRTK` | as configured on web-interface | Custom Message containing same Odometry information as well as status flags                                                                                   |
    | `/fixposition/poiimu`       | `sensor_msgs/Imu`         | as configured on web-interface | Bias Corrected acceleration and rotation rate in FP_POI                                                                                                       |
    | `/fixposition/ypr`          | `geometry_msgs/Vector3`   | as configured on web-interface | x = Yaw, y = Pitch, z = Roll in radian. Euler angles representation of rotation between ENU and P_POI. Only available after fusion initialization.            |

-   From **FP_A-LLH**, at the configured frequency (default 10Hz, output generator -> Fusion frequency):

    | Topic                    | Message Type            | Frequency                      | Description                    |
    | ------------------------ | ----------------------- | ------------------------------ | ------------------------------ |
    | `/fixposition/navsatfix` | `sensor_msgs/NavSatFix` | as configured on web-interface | Latitude, Longitude and Height |

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
    | Frames             | Topic        | Message needed to be selected on web-interface | Frequency                      |
    | ------------------ | ------------ | ---------------------------------------------- | ------------------------------ |
    | `ECEF-->FP_POI`    | `/tf`        | `ODOMETRY`                                     | as configured on web-interface |
    | `ECEF-->FP_ENU`    | `/tf`        | `ODOMETRY`                                     | as configured on web-interface |
    | `ECEF-->FP_ENU0`   | `/tf`        | `ODOMETRY`                                     | as configured on web-interface |
    | `FP_POI-->FP_IMUH` | `/tf`        | `ODOMETRY`                                     | 200Hz                          |
    | `FP_POI-->FP_VRTK` | `/tf_static` | `TF_POI_VRTK`                                  | 1Hz                            |
    | `FP_VRTK-->FP_CAM` | `/tf_static` | `TF_VRTK_CAM`                                  | 1Hz                            |

-   ROS TF Tree:

    ```mermaid
    graph TD;
    ECEF-->FP_POI-->FP_VRTK-->FP_CAM
    FP_POI-->FP_IMUH
    ECEF-->FP_ENU
    ECEF-->FP_ENU0
    ```

_Please note that the corresponding messages also has to be selected on the Fixposition V-RTK's configuration interface._

### Explaination of frame ids

| Frame ID    | Explaination                                                                                                                                   |
| ----------- | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| **ECEF**    | Earth-Center-Earth-Fixed frame.                                                                                                                |
| **FP_VRTK** | The coordinate frame on the V-RTK's housing on the Fixposition-Logo "X".                                                                       |
| **FP_POI**  | Point-Of-Interest, configured from V-RTK's web-interface with respect to the FP_VRTK frame. By default it is the same as FP_VRTK.              |
| **FP_ENU**  | The **local** East-North-Up coordinate frame with the origin at the same location as FP_POI.                                                   |
| **FP_ENU0** | The **global fixed** East-North-Up coordinate frame with the origin at the first received ODOMETRY position. Needed for visualization in Rviz. |
| **FP_CAM**  | The camera coordinate frame of the V-RTK.                                                                                                      |
| **FP_IMUH** | A local horizontal frame with the origin at the same location as FP_POI. This frame is a rough estimate determined by the IMU alone.           |

## Input Wheelspeed through the driver

The fp_ros_driver supports inputting a Speed msg (`msg/Speed.msg`) through the `/fixposition/speed` topic. The Speed msg is defined as a vector of WheelSensor msgs (`msg/WheelSensor.msg`). This message in turn, is intended to be a simplified version of the FP_B-MEASUREMENTS, containing three integers (vx, vy, and vz), three booleans (validity of the three velocity integers), and a string indicating the sensor from which the measurement originates. The integer velocity values should be in [mm/s], to have enough precision when being converted into an integer format.

Internally, upon arriving to the ros driver, wheelspeed measurements are converted into a full FP_B-MEASUREMENTS message, and sent via the TCP or serial interface to the Vision-RTK2, where they will be further processed. For more details regarding the definition FP_B-MEASUREMENTS message, please refer to [the following page](https://docs.fixposition.com/fd/fp_b-measurements), or to the VRTK2 integration manual.

## Code Documentation

Run `doxygen Doxyfile` to generate Doxygen code documentation.

## Fixposition ASCII messages

_This is an exerpt from the Integration Manual_

### Message structure

NMEA style framing is used. Frames (messages) are in this form:

<code><b style="color: red;">$</b><b style="color: green;">FP</b>,<b style="color: blue;">msg_type</b>,<b style="color: blue;">msg_version</b>,<em>field<sub>3</sub></em>,<em>field<sub>4</sub></em>,...,<em>field<sub>N</sub></em><b style="color: red;">\*CC</b><b style="color: red;">\r\n</b></code>

Where:

-   The NMEA style framing:
    -   <code><b style="color: red;">\$</b></code>
        -- Start character ("$", ASCII 36)
    -   <code><b style="color: red;">\*CC</b></code>
        -- Checksum: "\*" (ASCII 42) and two digit XOR value of all payload
        characters in captial hexadecimal notation, for example:
        "FPX" = `'F' ^ 'P' ^ 'X' = 70 ^ 80 ^ 88 = 78 = 0x4e` = checksum `4E`
    -   <code><b style="color: red;">\r\n</b></code>
        -- Sentence termination characters (CR + LF, ASCII 13 + 10)
-   A Fixposition identifier:
    -   <code><b style="color: green;">FP</b></code>
        -- Fixposition ASCII message identifier, "FP" (ASCII 70 + 80)
-   Fixposition message type and version:
    -   <code><b style="color: blue;">msg_type</b></code> (= <code><em>field<sub>1</sub></em></code>)
        -- Message type, all captial letters (ASCII 65--90)
    -   <code><b style="color: blue;">msg_version</b></code> (= <code><em>field<sub>2</sub></em></code>)
        -- Message version, decimal number (letters 0--9, ASCII 48--57), range 1--...
-   Data fields (payload)
    -   <code><em>field<sub>3</sub></em>,<em>field<sub>4</sub></em>,...,<em>field<sub>N</sub></em></code>
        -- The structure of the message data is defined by the <code><b style="color: blue;">msg_type</b></code>
        and <code><b style="color: blue;">version</b></code>.
        Each field can contain all printable 7-bit ASCII characters (ASCII 32–126), excluding the
        reserved characters `!` (ASCII 33), `$` (ASCII 36), `*` (ASCII 42), `,` (ASCII 44),
        `\` (ASCII 92), `~` (ASCII 126).
-   Field separators
    -   All fields (identifier, message type, message version, data fields) are separated by a comma (`,`, ASCII 44)
-   Null fields
    -   Data fields can be _null_, meaning their value is absent to indicate that no data is
        available. The data for null fields is the empty string. For example:
        -   Definition: <code>...,<em>field<sub>i</sub></em>,<em>field<sub>i+1</sub></em>,<em>field<sub>i+2</sub></em>,...</code>
        -   Values: <code><em>field<sub>i</sub></em></code> = 123, <code><em>field<sub>i+1</sub></em></code> = _null_,
            <code><em>field<sub>i+2</sub></em></code> = 456
        -   Payload string: <code>...,123,,456,...</code>
-   Data field types:
    -   _Numeric_: Decimal integer number, one or more digits (0-9) and optional leading "-" sign
    -   _Float (.x)_: Decimal floating point number, one or more digits (0-9) and optional leading "-" sign, with
        _x_ digits fractional part separated by a dot (".")
    -   _Float (x)_: Decimal floating point number with _x_ significant digits, optional leading "-", optional fractional
        part separated by a dot (".")
    -   _String_: String of allowed payload characters (but not the `,` field separator)
    -   ...
    -   ...

### ODOMETRY message

This message contains full fusion odometry output and additional status information. It is output at
the configured rate.

Example message (wrapped on multiple lines for readability):

    $FP,ODOMETRY,2,2231,227610.750000,4279243.1641,635824.2171,4671589.8683,-0.412792,0.290804,-0.123898,0.854216,
    -17.1078,-0.0526,-0.3252,0.02245,0.00275,0.10369,-1.0385,-1.3707,9.8249,4,1,8,8,1,
    0.01761,0.02274,0.01713,-0.00818,0.00235,0.00129,0.00013,0.00015,0.00014,-0.00001,0.00001,0.00002,
    0.03482,0.06244,0.05480,0.00096,0.00509,0.00054,fp_release_vr2_2.54.0_160*4F\r\n

Message fields:

|    # | Field                | Format     | Unit                        | Example                     | Description                                                            |
| ---: | -------------------- | ---------- | --------------------------- | --------------------------- | ---------------------------------------------------------------------- |
|    1 | `msg_type`           | String     | -                           | `ODOMETRY`                  | Message type, always `ODOMETRY` for this message                       |
|    2 | `msg_version`        | Numeric    | -                           | `2`                         | Message version, always `2` for this version of the `ODOMETRY` message |
|    3 | `gps_week`           | Numeric    | -                           | `2231`                      | GPS week number, range 0--9999                                         |
|    4 | `gps_tow`            | Float (.6) | s                           | `227610.750000`             | GPS time of week, range 0.000--604799.999999                           |
|    5 | `pos_x`              | Float (.4) | m                           | `4279243.1641`              | Position in ECEF, X component                                          |
|    6 | `pos_y`              | Float (.4) | m                           | `635824.2171`               | Position in ECEF, Y component                                          |
|    7 | `pos_z`              | Float (.4) | m                           | `4671589.8683`              | Position in ECEF, Z component                                          |
|    8 | `orientation_w`      | Float (.6) | -                           | `-0.412792`                 | Quaternion with respect to ECEF, W component                           |
|    9 | `orientation_x`      | Float (.6) | -                           | `0.290804`                  | Quaternion with respect to ECEF, X component                           |
|   10 | `orientation_y`      | Float (.6) | -                           | `-0.123898`                 | Quaternion with respect to ECEF, Y component                           |
|   11 | `orientation_z`      | Float (.6) | -                           | `0.854216`                  | Quaternion with respect to ECEF, Z component                           |
|   12 | `vel_x`              | Float (.4) | m/s                         | `-17.1078`                  | Velocity in output frame, X component                                  |
|   13 | `vel_y`              | Float (.4) | m/s                         | `-0.0526`                   | Velocity in output frame, Y component                                  |
|   14 | `vel_z`              | Float (.4) | m/s                         | `-0.3252`                   | Velocity in output frame, Z component                                  |
|   15 | `rot_x`              | Float (.5) | rad/s                       | `0.02245`                   | Bias corrected angular velocity in output frame, X component           |
|   16 | `rot_y`              | Float (.5) | rad/s                       | `0.00275`                   | Bias corrected angular velocity in output frame, Y component           |
|   17 | `rot_z`              | Float (.5) | rad/s                       | `0.10369`                   | Bias corrected angular velocity in output frame, Z component           |
|   18 | `acc_x`              | Float (.4) | m/s<sup>2</sup>             | `-1.0385`                   | Bias corrected acceleration in output frame, X component               |
|   19 | `acc_y`              | Float (.4) | m/s<sup>2</sup>             | `-1.3707`                   | Bias corrected acceleration in output frame, Y component               |
|   20 | `acc_z`              | Float (.4) | m/s<sup>2</sup>             | `9.8249`                    | Bias corrected acceleration in output frame, Z component               |
|   21 | `fusion_status`      | Numeric    | -                           | `4`                         | Fustion status, see below                                              |
|   22 | `imu_bias_status`    | Numeric    | -                           | `1`                         | IMU bias status, see below                                             |
|   23 | `gnss1_fix`          | Numeric    | -                           | `8`                         | Fix status of GNSS1 receiver, see below                                |
|   24 | `gnss2_fix`          | Numeric    | -                           | `8`                         | Fix status of GNSS2 receiver, see below                                |
|   25 | `wheelspeed_status`  | Numeric    | -                           | `1`                         | Wheelspeed status, see below                                           |
|   26 | `pos_cov_xx`         | Float (5)  | m<sup>2</sup>               | `0.01761`                   | Position covariance, element XX                                        |
|   27 | `pos_cov_yy`         | Float (5)  | m<sup>2</sup>               | `0.02274`                   | Position covariance, element YY                                        |
|   28 | `pos_cov_zz`         | Float (5)  | m<sup>2</sup>               | `0.01713`                   | Position covariance, element ZZ                                        |
|   29 | `pos_cov_xy`         | Float (5)  | m<sup>2</sup>               | `-0.00818`                  | Position covariance, element XY                                        |
|   30 | `pos_cov_yz`         | Float (5)  | m<sup>2</sup>               | `0.00235`                   | Position covariance, element YZ                                        |
|   31 | `pos_cov_xz`         | Float (5)  | m<sup>2</sup>               | `0.00129`                   | Position covariance, element XZ                                        |
|   32 | `orientation_cov_xx` | Float (5)  | rad<sup>2</sup>             | `0.00013`                   | Velocity covariance, element XX                                        |
|   33 | `orientation_cov_yy` | Float (5)  | rad<sup>2</sup>             | `0.00015`                   | Velocity covariance, element YY                                        |
|   34 | `orientation_cov_zz` | Float (5)  | rad<sup>2</sup>             | `0.00014`                   | Velocity covariance, element ZZ                                        |
|   35 | `orientation_cov_xy` | Float (5)  | rad<sup>2</sup>             | `-0.00001`                  | Velocity covariance, element XY                                        |
|   36 | `orientation_cov_yz` | Float (5)  | rad<sup>2</sup>             | `0.00001`                   | Velocity covariance, element YZ                                        |
|   37 | `orientation_cov_xz` | Float (5)  | rad<sup>2</sup>             | `0.00002`                   | Velocity covariance, element XZ                                        |
|   38 | `vel_cov_xx`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `0.03482`                   | Velocity covariance, element XX                                        |
|   39 | `vel_cov_yy`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `0.06244`                   | Velocity covariance, element YY                                        |
|   40 | `vel_cov_zz`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `0.05480`                   | Velocity covariance, element ZZ                                        |
|   41 | `vel_cov_xy`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `0.00096`                   | Velocity covariance, element XY                                        |
|   42 | `vel_cov_yz`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `0.00509`                   | Velocity covariance, element YZ                                        |
|   43 | `vel_cov_xz`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `0.00054`                   | Velocity covariance, element XZ                                        |
|   44 | `sw_version`         | String     | -                           | `fp_release_vr2_2.54.0_160` | Software version                                                       |

Fusion status (`fusion_status`):

| Value | Description                 |
| :---: | --------------------------- |
|  `0`  | Not started                 |
|  `1`  | Vision only                 |
|  `2`  | Visual inertial fusion      |
|  `3`  | Inertial-GNSS fusion        |
|  `4`  | Visual-inertial-GNSS fusion |

IMU bias status (`imu_bias_status`):

| Value | Description        |
| :---: | ------------------ |
|  `0`  | Not converged      |
|  `1`  | IMU bias converged |

GNSS fix type (`gnss1_fix`, `gnss2_fix`):

| Value | Description                       |
| :---: | --------------------------------- |
|  `0`  | Unknown                           |
|  `1`  | No fix                            |
|  `2`  | Dead-reckoning only               |
|  `3`  | Time-only fix                     |
|  `4`  | Single 2D fix                     |
|  `5`  | Single 3D fix                     |
|  `6`  | Single 3D fix with dead-reckoning |
|  `7`  | RTK float fix                     |
|  `8`  | RTK fixed fix                     |

Wheelspeed status (`wheelspeed_status`):

| Value | Description                                              |
| :---: | -------------------------------------------------------- |
| `-1`  | No wheelspeed enabled                                    |
|  `0`  | At least one wheelspeed enabled, no wheelspeed converged |
|  `1`  | At least one wheelspeed enabled and converged            |

Remarks:

-   The output frame is the frame configured on the web-interface.

### LLH message

This message contains time, geographic coordinates and the position covariance of the output frame
in East-North-up (ENU). The coordinates are transformed from ECEF using the WGS-84 parameters
(see also [[coordinates#vrtk-output-coordinate-system]]). It is output at the configured rate.

Example message (wrapped on multiple lines for readability):

    $FP,LLH,1,2231,227563.250000,47.392357470,8.448121451,473.5857,
    0.04533,0.03363,0.02884,0.00417,0.00086,-0.00136*62\r\n

Message fields:

|    # | Field         | Format     | Unit          | Example         | Description                                                                |
| ---: | ------------- | ---------- | ------------- | --------------- | -------------------------------------------------------------------------- |
|    1 | `msg_type`    | String     | -             | `LLH`           | Message type, always `LLH` for this message                                |
|    2 | `msg_version` | Numeric    | -             | `1`             | Message version, always `1` for this version of the `LLH` message          |
|    3 | `gps_week`    | Numeric    | -             | `2231`          | GPS week number, range 0--9999                                             |
|    4 | `gps_tow`     | Float (.6) | s             | `227563.250000` | GPS time of week, range 0.000--604799.999999                               |
|    5 | `latitude`    | Float (.9) | deg           | `47.392357470`  | Latitude, range -90.000000000--90.000000000, > 0 for North, < 0 for South  |
|    6 | `longitude`   | Float (.9) | deg           | `8.448121451`   | Longitude, range -180.000000000--180.000000000, > 0 for East, < 0 for West |
|    7 | `height`      | Float (.4) | m             | `473.5857`      | Ellipsoidal height, range -1000.0000-50000.0000                            |
|    8 | `pos_cov_ee`  | Float (5)  | m<sup>2</sup> | `0.04533`       | Position covariance in ENU, element EE                                     |
|    9 | `pos_cov_nn`  | Float (5)  | m<sup>2</sup> | `0.03363`       | Position covariance in ENU, element NN                                     |
|   10 | `pos_cov_uu`  | Float (5)  | m<sup>2</sup> | `0.02884`       | Position covariance in ENU, element UU                                     |
|   11 | `pos_cov_en`  | Float (5)  | m<sup>2</sup> | `0.00417`       | Position covariance in ENU, element EN                                     |
|   12 | `pos_cov_nu`  | Float (5)  | m<sup>2</sup> | `0.00086`       | Position covariance in ENU, element NU                                     |
|   13 | `pos_cov_eu`  | Float (5)  | m<sup>2</sup> | `-0.00136`      | Position covariance in ENU, element EU                                     |

### RAWIMU message

This message contains time, acceleration and angular velocity (raw value, no bias correction, only
coordinate transformation applied) in the vrtk frame - the X on the sensor. See also
[[coordinates#vrtk-output-coordinate-system]]. It is output at 200Hz IMU frequency regardless
whether fusion is running or not.

Example message:

    $FP,RAWIMU,1,2197,126191.777855,-0.199914,0.472851,9.917973,0.023436,0.007723,0.002131*34\r\n

Message fields:

|    # | Field         | Format     | Unit            | Example         | Description                                                          |
| ---: | ------------- | ---------- | --------------- | --------------- | -------------------------------------------------------------------- |
|    1 | `msg_type`    | String     | -               | `RAWIMU`        | Message type, always `RAWIMU` for this message                       |
|    2 | `msg_version` | Numeric    | -               | `1`             | Message version, always `1` for this version of the `RAWIMU` message |
|    3 | `gps_week`    | Numeric    | -               | `2197`          | GPS week number, range 0--9999                                       |
|    4 | `gps_tow`     | Float (.6) | s               | `126191.777855` | GPS time of week, range 0.000--604799.999999                         |
|    5 | `acc_x`       | Float (.6) | m/s<sup>2</sup> | `-0.199914`     | Raw acceleration in output frame, X component                        |
|    6 | `acc_y`       | Float (.6) | m/s<sup>2</sup> | `0.472851`      | Raw acceleration in output frame, Y component                        |
|    7 | `acc_z`       | Float (.6) | m/s<sup>2</sup> | `9.917973`      | Raw acceleration in output frame, Z component                        |
|    8 | `rot_x`       | Float (.6) | rad/s           | `0.023436`      | Raw angular velocity in output frame, X component                    |
|    9 | `rot_y`       | Float (.6) | rad/s           | `0.007723`      | Raw angular velocity in output frame, Y component                    |
|   10 | `rot_z`       | Float (.6) | rad/s           | `0.002131`      | Raw angular velocity in output frame, Z component                    |

### CORRIMU message

This message contains time, acceleration and angular velocity (coordinate transformation and bias
correction applied) in the vrtk frame - the X on the sensor. See also
[[coordinates#vrtk-output-coordinate-system]]. It is output at 200Hz IMU frequency, but only when
fusion is initialized and IMU biases is converged.

Example message:

    $FP,CORRIMU,1,2197,126191.777855,-0.195224,0.393969,9.869998,0.013342,-0.004620,-0.000728*7D\r\n

Message fields:

|    # | Field         | Format     | Unit            | Example         | Description                                                          |
| ---: | ------------- | ---------- | --------------- | --------------- | -------------------------------------------------------------------- |
|    1 | `msg_type`    | String     | -               | `CORRIMU`       | Message type, always `RAWIMU` for this message                       |
|    2 | `msg_version` | Numeric    | -               | `1`             | Message version, always `1` for this version of the `RAWIMU` message |
|    3 | `gps_week`    | Numeric    | -               | `2197`          | GPS week number, range 0--9999                                       |
|    4 | `gps_tow`     | Float (.6) | s               | `126191.777855` | GPS time of week, range 0.000--604799.999                            |
|    5 | `acc_x`       | Float (.6) | m/s<sup>2</sup> | `-0.195224`     | Raw acceleration in output frame, X component                        |
|    6 | `acc_y`       | Float (.6) | m/s<sup>2</sup> | `0.393969`      | Raw acceleration in output frame, Y component                        |
|    7 | `acc_z`       | Float (.6) | m/s<sup>2</sup> | `9.869998`      | Raw acceleration in output frame, Z component                        |
|    8 | `rot_x`       | Float (.6) | rad/s           | `0.013342`      | Raw angular velocity in output frame, X component                    |
|    9 | `rot_y`       | Float (.6) | rad/s           | `-0.004620`     | Raw angular velocity in output frame, Y component                    |
|   10 | `rot_z`       | Float (.6) | rad/s           | `-0.000728`     | Raw angular velocity in output frame, Z component                    |

Remarks:

-   The output frame of the IMU messages is the X on VRTK sensor, NOT the frame configured from the
    webinterface (They are of course the same when on webinterface the configs are 0s).

### TF message

This message contains information for static coordinate transformations.

Example messages:

    $FP,TF,2,2233,315835.000000,VRTK,CAM,-0.00000,-0.00000,-0.00000,1.000000,0.000000,0.000000,0.000000*6B\r\n
    $FP,TF,2,2233,315835.000000,POI,VRTK,-0.99301,-2.01395,-2.99298,0.999995,-0.002616,-0.001748,-0.000868*52\r\n

Message fields:

|    # | Field           | Format     | Unit | Example         | Description                                                      |
| ---: | --------------- | ---------- | ---- | --------------- | ---------------------------------------------------------------- |
|    1 | `msg_type`      | String     | -    | `TF`            | Message type, always `TF` for this message                       |
|    2 | `msg_version`   | Numeric    | -    | `2`             | Message version, always `2` for this version of the `TF` message |
|    3 | `gps_week`      | Numeric    | -    | `2233`          | GPS week number, range 0--9999                                   |
|    4 | `gps_tow`       | Float (.6) | s    | `315835.000000` | GPS time of week, range 0.000--604799.999999                     |
|    5 | `frame_a`       | String     | -    | `POI`           | Target frame (maximum 8 characters: A-Z and 0-9)                 |
|    6 | `frame_b`       | String     | -    | `VRTK`          | Initial frame (maximum 8 characters: A-Z and 0-9)                |
|    7 | `translation_x` | Float (.5) | m    | `-0.99301`      | Translation, X component                                         |
|    8 | `translation_y` | Float (.5) | m    | `-2.01395`      | Translation, Y component                                         |
|    9 | `translation_z` | Float (.5) | m    | `-2.99298`      | Translation, Z component                                         |
|   10 | `orientation_w` | Float (.6) | -    | `0.999995`      | Rotation in quaternion, W component                              |
|   11 | `orientation_x` | Float (.6) | -    | `-0.002616`     | Rotation in quaternion, X component                              |
|   12 | `orientation_y` | Float (.6) | -    | `-0.001748`     | Rotation in quaternion, Y component                              |
|   13 | `orientation_z` | Float (.6) | -    | `-0.000868`     | Rotation in quaternion, Z component                              |

# Fixposition Odometry Converter

This is an extra node is provided to help with the integration of the wheel odometry on your vehicle. For details, see the subfolder [fixposition_odometry_converter_ros1](fixposition_odometry_converter_ros1/README.md) (ROS 1) and [fixposition_odometry_converter_ros2](fixposition_odometry_converter_ros2/README.md) (ROS 2). When building the ROS 1 version add a file named 'CATKIN_IGNORE' to the `fixposition_odometry_converter_ros2` folder.

# License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
