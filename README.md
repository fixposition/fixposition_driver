# Fixposition Driver

[ROS](https://www.ros.org/) Driver for [Fixposition Vision-RTK 2](https://www.fixposition.com/product)

## Dependencies

-  [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), tested with version [3.3.7](https://gitlab.com/libeigen/eigen/-/releases/3.3.7)
-  [Boost](https://www.boost.org/), tested with version [1.65.0](https://www.boost.org/users/history/version_1_65_0.html)
-  [CMake](https://cmake.org/)
-  [Catkin](http://wiki.ros.org/catkin)
-  [Trransforms] (http://wiki.ros.org/tf)

-  **[fixposition_gnss_tf](https://github.com/fixposition/fixposition_gnss_tf)**: Fixposition GNSS Transformation Lib, minimum version **2.0.0**

This driver operates as a ROS node, connecting to either a TCP or serial stream of Fixposition Vision-RTK output data, see [Fixposition ASCII messages](#fixposition-ascii-messages) and the **Integration Manual**.

## Installing dependencies on Ubuntu system

```
 sudo apt update
 sudo apt install -y wget
 wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
 sudo apt-get update
 sudo apt-get install -y python3-catkin-tools
 sudo apt install -y build-essential cmake
 sudo apt install -y libeigen3-dev
 sudo apt install -y ros-melodic-ros-tutorials ros-melodic-geometry-tutorials ros-melodic-rviz ros-melodic-rosbash ros-melodic-rqt-tf-tree
```


## Installation

To install the node, extract / clone the code and `fixposition_gnss_tf` to your catkin workspace's `src` folder:

```bash
# The folder structure should look like this
fp_public_ws
└── src
    ├── fixposition_driver
    └── gnsstransformationlib
```

and build it with:

`catkin build fixposition_driver`

Then source your development environment:

`source devel/setup.bash`

## Launch the Driver

-  To launch the node in serial mode, run:

   `roslaunch fixposition_driver serial.launch`

-  In TCP mode (Wi-Fi):

   `roslaunch fixposition_driver tcp.launch`

-  In TCP mode (Ethernet):

   `roslaunch fixposition_driver tcp.launch`

To change the settings of TCP (IP, Port) or Serial (Baudrate, Port) connections, check the `launch/tcp.yaml` and `launch/serial.yaml` files.

## Input Wheelspeed through the driver

The fp_ros_driver support inputing a Speed msg (`msg/Speed.msg`) through the `/fixposition/speed` topic.

The input velocity values should be in [mm/s], respectively [mrad/s], as integer 32bit. There are 2 Options:

-  Option 1: Only one vehicle speed, then only fill a single value as the vehicle speed
-  Option 2: One vehicle speed and the rotation around the vehicle's rotation center
-  Option 3: Fill in 4 Values of 4 wheels, in the order of FR, FL, RR, RL

The input values will be converted into a RAWDMI message and sent via the TCP interface to the Vision-RTK2, where it will be further processed and added into the positioning engine. The following protocol is used when filling the DMI messages, as per the documentation:

| Offset                               | Type      | Value       | Description |
|-------------------------------------:|-----------|-------------|-------------|
|     0  | uint8_t      | `0xaa`      | (header) Sync byte 1 |
|     1  | uint8_t      | `0x44`      | (header) Sync byte 2 |
|     2  | uint8_t      | `0x13`      | (header) Sync byte 3 |
|     3  | uint8_t      | `20`        | (header) Payload length |
|     4  | uint16_t     | `2269`      | (header) Message ID |
|     6  | uint16_t     | `0`         | (header) Week number, ignored by VRTK2 |
|     8  | int32_t      | `0`         | (header) Time of week [ms], ignored by VRTK2 |
|    12  | int32_t      | *dmi1*      | (payload) Speed value 1, for RC or FR wheel |
|    16  | int32_t      | *dmi2*      | (payload) Speed value 2, for FL wheel or YW sensor |
|    20  | int32_t      | *dmi3*      | (payload) Speed value 3, for RR wheel |
|    24  | int32_t      | *dmi4*      | (payload) Speed value 4, for RL wheel |
|    28  | uint32_t     | *mask*      | (payload) Bitfield Speed value mask |
|        | bit 0        | *dmi1_mask* | (payload) Validity bit for *dmi1* value |
|        | bit 1        | *dmi2_mask* | (payload) Validity bit for *dmi2* value |
|        | bit 2        | *dmi3_mask* | (payload) Validity bit for *dmi3* value |
|        | bit 3        | *dmi4_mask* | (payload) Validity bit for *dmi4* value |
|        | bits 10…4    | *dmi1_type* | (payload) Type of measurement present in *dmi1* value |
|        | bits 17…11   | *dmi2_type* | (payload) Type of measurement present in *dmi2* value |
|        | bits 24…18   | *dmi3_type* | (payload) Type of measurement present in *dmi3* value |
|        | bits 31…25   | *dmi3_type* | (payload) Type of measurement present in *dmi3* value |
|    32  | uint32_t     | *cksum*     | (checksum) CRC2 checksum (seed 0x00000000, polynomial 0xEDB88320) |

Note: _Currently the wheelspeed input through the ROS driver is only supported in the TCP mode_

## Output of the driver

### Messages and TF tree

The output is published on the following:

-  From ODOMETRY, at the configured frequency
   -  Messages

   | Topic                       | Message Type              | Frequency                      | Description                                                                                           |
   | --------------------------- | ------------------------- | ------------------------------ | ----------------------------------------------------------------------------------------------------- |
   | `/fixposition/odometry`     | `nav_msgs/Odometry`       | as configured on web-interface | Position, Orientation from ECEF to FP_POI, Velocity and Angular Velocity in FP_POI                    |
   | `/fixposition/odometry_enu` | `nav_msgs/Odometry`       | as configured on web-interface | Position, Orientation from ECEF to ENU0, Velocity and Angular Velocity in FP_POI                      |
   | `/fixposition/ypr`          | `geometry_msgs/Vector3`   | as configured on web-interface | x = Yaw, y = Pitch, z = Roll in radian. Euler angles representation of rotation between ENU and P_POI. Only available after fusion initialization.       |
   | `/fixposition/imu_ypr`      | `geometry_msgs/Vector3`   | 200Hz                          | x = 0.0, y = Pitch, z = Roll in radian. Euler angles representation of rotation between a local horizontal frame and P_POI. Rough estimation using IMU alone. |
   | `/fixposition/vrtk`         | `fixposition_driver/VRTK` | as configured on web-interface | Custom Message containing same Odometry information as well as status flags                           |
   | `/fixposition/poiimu`       | `sensor_msgs/Imu`         | as configured on web-interface | Bias Corrected acceleration and rotation rate in FP_POI                                               |

-  From LLH, at the configured frequency


   | Topic                    | Message Type            | Frequency                      | Description                    |
   | ------------------------ | ----------------------- | ------------------------------ | ------------------------------ |
   | `/fixposition/navsatfix` | `sensor_msgs/NavSatFix` | as configured on web-interface | Latitude, Longitude and Height |


-  From RAWIMU, at 200Hz

   | Topic                 | Message Type      | Frequency | Description                                                                               |
   | --------------------- | ----------------- | --------- | ----------------------------------------------------------------------------------------- |
   | `/fixposition/rawimu` | `sensor_msgs/Imu` | 200Hz     | Raw (without bias correction) IMU acceleration and angular velocity data in FP_VRTK frame |


-  From CORRIMU, at 200Hz

   | Topic                  | Message Type      | Frequency | Description                                                                |
   | ---------------------- | ----------------- | --------- | -------------------------------------------------------------------------- |
   | `/fixposition/corrimu` | `sensor_msgs/Imu` | 200Hz     | Bias Corrected IMU acceleration and angular velocity data in FP_VRTK frame |

-  TFs:
    | Frames                       | Topic        | Message needed to be selected on web-interface | Frequency                      |
    | ------------------           | ------------ | ---------------------------------------------- | ------------------------------ |
    | `ECEF-->FP_POI`              | `/tf`        | `ODOMETRY`                                     | as configured on web-interface |
    | `ECEF-->FP_ENU`              | `/tf`        | `ODOMETRY`                                     | as configured on web-interface |
    | `ECEF-->FP_ENU0`             | `/tf`        | `ODOMETRY`                                     | as configured on web-interface |
    | `FP_POI-->FP_IMU_HORIZONTAL` | `/tf`        | `ODOMETRY`                                     | 200Hz                          |
    | `FP_POI-->FP_VRTK`           | `/tf_static` | `TF_POI_VRTK`                                  | 1Hz                            |
    | `FP_VRTK-->FP_CAM`           | `/tf_static` | `TF_VRTK_CAM`                                  | 1Hz                            |


-  ROS TF Tree:

   ```mermaid
   graph TD;
   ECEF-->FP_POI-->FP_VRTK-->FP_CAM
   FP_POI-->FP_IMU_HORIZONTAL
   ECEF-->FP_ENU
   ECEF-->FP_ENU0
   ```

_Please note that the corresponding messages also has to be selected on the Fixposition V-RTK's configuration interface._

### Explaination of frame ids

| Frame ID    | Explaination                                                                                                                                    |
| ----------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| **ECEF**    | Earth-Center-Earth-Fixed frame.                                                                                                                 |
| **FP_VRTK** | The coordinate frame on the V-RTK's housing on the Fixposition-Logo "X".                                                                        |
| **FP_POI**  | Point-Of-Interest, configured from V-RTK's web-interface with respect to the FP_VRTK frame. By default it is the same as FP_VRTK.               |
| **FP_ENU**  | The **local** East-North-Up coordinate frame with the origin at the same location as FP_POI.                                                    |
| **FP_ENU0** | The **global fixed** East-North-Up coordinate frame with the origin at the first received ODOMETRY position. Needed for visualization in Rviz.  |
| **FP_CAM**  | The camera coordinate frame of the V-RTK.                                                                                                       |
| **FP_IMU_HORIZONTAL**  | A local horizontal frame with the origin at the same location as FP_POI. This frame is a rough estimate determined by the IMU alone. |

## Code Documentation

Run `doxygen Doxyfile` to generate Doxygen code documentation.

## Fixposition ASCII messages

_This is an exerpt from the Integration Manual_

### Message structure

NMEA style framing is used. Frames (messages) are in this form:

<code><b style="color: red;">$</b><b style="color: green;">FP</b>,<b style="color: blue;">msg_type</b>,<b style="color: blue;">msg_version</b>,<em>field<sub>3</sub></em>,<em>field<sub>4</sub></em>,...,<em>field<sub>N</sub></em><b style="color: red;">\*CC</b><b style="color: red;">\r\n</b></code>

Where:

-  The NMEA style framing:
   -  <code><b style="color: red;">\$</b></code>
      -- Start character ("$", ASCII 36)
   -  <code><b style="color: red;">\*CC</b></code>
      -- Checksum: "\*" (ASCII 42) and two digit XOR value of all payload
      characters in captial hexadecimal notation, for example:
      "FPX" = `'F' ^ 'P' ^ 'X' = 70 ^ 80 ^ 88 = 78 = 0x4e` = checksum `4E`
   -  <code><b style="color: red;">\r\n</b></code>
      -- Sentence termination characters (CR + LF, ASCII 13 + 10)
-  A Fixposition identifier:
   -  <code><b style="color: green;">FP</b></code>
      -- Fixposition ASCII message identifier, "FP" (ASCII 70 + 80)
-  Fixposition message type and version:
   -  <code><b style="color: blue;">msg_type</b></code> (= <code><em>field<sub>1</sub></em></code>)
      -- Message type, all captial letters (ASCII 65--90)
   -  <code><b style="color: blue;">msg_version</b></code> (= <code><em>field<sub>2</sub></em></code>)
      -- Message version, decimal number (letters 0--9, ASCII 48--57), range 1--...
-  Data fields (payload)
   -  <code><em>field<sub>3</sub></em>,<em>field<sub>4</sub></em>,...,<em>field<sub>N</sub></em></code>
      -- The structure of the message data is defined by the <code><b style="color: blue;">msg_type</b></code>
      and <code><b style="color: blue;">version</b></code>.
      Each field can contain all printable 7-bit ASCII characters (ASCII 32–126), excluding the
      reserved characters `!` (ASCII 33), `$` (ASCII 36), `*` (ASCII 42), `,` (ASCII 44),
      `\` (ASCII 92), `~` (ASCII 126).
-  Field separators
   -  All fields (identifier, message type, message version, data fields) are separated by a comma (`,`, ASCII 44)
-  Null fields
   -  Data fields can be _null_, meaning their value is absent to indicate that no data is
      available. The data for null fields is the empty string. For example:
      -  Definition: <code>...,<em>field<sub>i</sub></em>,<em>field<sub>i+1</sub></em>,<em>field<sub>i+2</sub></em>,...</code>
      -  Values: <code><em>field<sub>i</sub></em></code> = 123, <code><em>field<sub>i+1</sub></em></code> = _null_,
         <code><em>field<sub>i+2</sub></em></code> = 456
      -  Payload string: <code>...,123,,456,...</code>
-  Data field types:
   -  _Numeric_: Decimal integer number, one or more digits (0-9) and optional leading "-" sign
   -  _Float (.x)_: Decimal floating point number, one or more digits (0-9) and optional leading "-" sign, with
      _x_ digits fractional part separated by a dot (".")
   -  _Float (x)_: Decimal floating point number with _x_ significant digits, optional leading "-", optional fractional
      part separated by a dot (".")
   -  _String_: String of allowed payload characters (but not the `,` field separator)
   -  ...
   -  ...

### ODOMETRY message

This message contains full fusion odometry output and additional status information. It is output at
the configured rate.

Example message (wrapped on multiple lines for readability):

    $FP,ODOMETRY,1,2197,126191.765,4278415.1169,636245.1942,4672227.8942,-0.921035,-0.001266,-0.365401,
    -0.134863,0.6169,-0.0140,-0.0068,0.01857,-0.01427,-0.00746,-0.1185,-0.0795,9.7791,4,1,1,1,0.55214,
    0.33578,0.50777,0.08625,-0.13062,-0.45209,0.00227,0.00020,0.00270,0.00027,0.00031,0.00232,
    0.03314,0.03828,0.03199,-0.00290,0.00246,-0.00119,fp_release_vr2_2.36.1_67*47

Message fields:

|    # | Field                | Format     | Unit                        | Example                    | Description                                                            |
| ---: | -------------------- | ---------- | --------------------------- | -------------------------- | ---------------------------------------------------------------------- |
|    1 | `msg_type`           | String     | -                           | `ODOMETRY`                 | Message type, always `ODOMETRY` for this message                       |
|    2 | `msg_version`        | Numeric    | -                           | `1`                        | Message version, always `1` for this version of the `ODOMETRY` message |
|    3 | `gps_week`           | Numeric    | -                           | `2197`                     | GPS week number, range 0--9999                                         |
|    4 | `gps_tow`            | Float (.6) | s                           | `126191.765`               | GPS time of week, range 0.000--604799.999                              |
|    5 | `pos_x`              | Float (.4) | m                           | `4278415.1169`             | Position in ECEF, X component                                          |
|    6 | `pos_y`              | Float (.4) | m                           | `636245.1942`              | Position in ECEF, Y component                                          |
|    7 | `pos_z`              | Float (.4) | m                           | `4672227.8942`             | Position in ECEF, Z component                                          |
|    8 | `orientation_w`      | Float (.6) | -                           | `-0.921035`                | Quaternion with respect to ECEF, W component                           |
|    9 | `orientation_x`      | Float (.6) | -                           | `-0.001266`                | Quaternion with respect to ECEF, X component                           |
|   10 | `orientation_y`      | Float (.6) | -                           | `-0.365401`                | Quaternion with respect to ECEF, Y component                           |
|   11 | `orientation_z`      | Float (.6) | -                           | `-0.134863`                | Quaternion with respect to ECEF, Z component                           |
|   12 | `vel_x`              | Float (.4) | m/s                         | `0.6169`                   | Velocity in output frame, X component                                  |
|   13 | `vel_y`              | Float (.4) | m/s                         | `-0.0140`                  | Velocity in output frame, Y component                                  |
|   14 | `vel_z`              | Float (.4) | m/s                         | `-0.0068`                  | Velocity in output frame, Z component                                  |
|   15 | `rot_x`              | Float (.5) | rad/s                       | `0.01857`                  | Bias corrected angular velocity in output frame, X component           |
|   16 | `rot_y`              | Float (.5) | rad/s                       | `-0.01427`                 | Bias corrected angular velocity in output frame, Y component           |
|   17 | `rot_z`              | Float (.5) | rad/s                       | `-0.00746`                 | Bias corrected angular velocity in output frame, Z component           |
|   18 | `acc_x`              | Float (.4) | m/s<sup>2</sup>             | `-0.1185`                  | Bias corrected acceleration in output frame, X component               |
|   19 | `acc_y`              | Float (.4) | m/s<sup>2</sup>             | `-0.0795`                  | Bias corrected acceleration in output frame, Y component               |
|   20 | `acc_z`              | Float (.4) | m/s<sup>2</sup>             | `9.7791`                   | Bias corrected acceleration in output frame, Z component               |
|   21 | `fusion_status`      | Numeric    | -                           | `4`                        | Fustion status, see below                                              |
|   22 | `imu_bias_status`    | Numeric    | -                           | `1`                        | IMU bias status, see below                                             |
|   23 | `gnss_fix_type`      | Numeric    | -                           | `1`                        | GNSS fix type, see below                                               |
|   24 | `wheelspeed_status`  | Numeric    | -                           | `1`                        | Wheelspeed status, see below                                           |
|   25 | `pos_cov_xx`         | Float (5)  | m<sup>2</sup>               | `0.55214`                  | Position covariance, element XX                                        |
|   26 | `pos_cov_yy`         | Float (5)  | m<sup>2</sup>               | `0.33578`                  | Position covariance, element YY                                        |
|   27 | `pos_cov_zz`         | Float (5)  | m<sup>2</sup>               | `0.50777`                  | Position covariance, element ZZ                                        |
|   28 | `pos_cov_xy`         | Float (5)  | m<sup>2</sup>               | `0.08625`                  | Position covariance, element XY                                        |
|   29 | `pos_cov_yz`         | Float (5)  | m<sup>2</sup>               | `-0.13062`                 | Position covariance, element YZ                                        |
|   30 | `pos_cov_xz`         | Float (5)  | m<sup>2</sup>               | `-0.45209`                 | Position covariance, element XZ                                        |
|   31 | `orientation_cov_xx` | Float (5)  | rad<sup>2</sup>             | `0.00227`                  | Velocity covariance, element XX                                        |
|   32 | `orientation_cov_yy` | Float (5)  | rad<sup>2</sup>             | `0.00020`                  | Velocity covariance, element YY                                        |
|   33 | `orientation_cov_zz` | Float (5)  | rad<sup>2</sup>             | `0.00270`                  | Velocity covariance, element ZZ                                        |
|   34 | `orientation_cov_xy` | Float (5)  | rad<sup>2</sup>             | `0.00027`                  | Velocity covariance, element XY                                        |
|   35 | `orientation_cov_yz` | Float (5)  | rad<sup>2</sup>             | `0.00031`                  | Velocity covariance, element YZ                                        |
|   36 | `orientation_cov_xz` | Float (5)  | rad<sup>2</sup>             | `0.00232`                  | Velocity covariance, element XZ                                        |
|   37 | `vel_cov_xx`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `0.03314`                  | Velocity covariance, element XX                                        |
|   38 | `vel_cov_yy`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `0.03828`                  | Velocity covariance, element YY                                        |
|   39 | `vel_cov_zz`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `0.03199`                  | Velocity covariance, element ZZ                                        |
|   40 | `vel_cov_xy`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `-0.00290`                 | Velocity covariance, element XY                                        |
|   41 | `vel_cov_yz`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `0.00246`                  | Velocity covariance, element YZ                                        |
|   42 | `vel_cov_xz`         | Float (5)  | m<sup>2</sup>/s<sup>2</sup> | `-0.00119`                 | Velocity covariance, element XZ                                        |
|   43 | `sw_version`         | String     | -                           | `fp_release_vr2_2.36.1_67` | Software version                                                       |

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

Wheelspeed status (`wheelspeed_status`):

| Value | Description                                              |
| :---: | -------------------------------------------------------- |
| `-1`  | No wheelspeed enabled                                    |
|  `0`  | At least one wheelspeed enabled, no wheelspeed converged |
|  `1`  | At least one wheelspeed enabled and converged            |

GNSS fix type (`gnss_fix_type`):

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

Remarks:

-  The output frame is the frame configured on the web-interface.

### LLH message

This message contains time, geographic coordinates and the position covariance of the output frame
in East-North-up (ENU). The coordinates are transformed from ECEF using the WGS-84 parameters
(see also [[coordinates#vrtk-output-coordinate-system]]). It is output at the configured rate.

Example message (wrapped on multiple lines for readability):

    $FP,LLH,1,2197,126191.765,47.398826818,8.458494107,457.518,0.31537,
    1.0076,0.072696,-0.080012,0.0067274,-0.011602*4E\r\n

Message fields:

|    # | Field         | Format     | Unit          | Example        | Description                                                                |
| ---: | ------------- | ---------- | ------------- | -------------- | -------------------------------------------------------------------------- |
|    1 | `msg_type`    | String     | -             | `LLH`          | Message type, always `LLH` for this message                                |
|    2 | `msg_version` | Numeric    | -             | `1`            | Message version, always `1` for this version of the `LLH` message          |
|    3 | `gps_week`    | Numeric    | -             | `2197`         | GPS week number, range 0--9999                                             |
|    4 | `gps_tow`     | Float (.6) | s             | `126191.765`   | GPS time of week, range 0.000--604799.999                                  |
|    5 | `latitude`    | Float (.9) | deg           | `47.398826818` | Latitude, range -90.000000000--90.000000000, > 0 for North, < 0 for South  |
|    6 | `longitude`   | Float (.9) | deg           | `8.458494107`  | Longitude, range -180.000000000--180.000000000, > 0 for East, < 0 for West |
|    7 | `height`      | Float (.4) | m             | `457.518`      | Ellipsoidal height                                                         |
|    8 | `pos_cov_ee`  | Float (5)  | m<sup>2</sup> | `0.31537`      | Position covariance in ENU, element EE                                     |
|    9 | `pos_cov_nn`  | Float (5)  | m<sup>2</sup> | `1.0076`       | Position covariance in ENU, element NN                                     |
|   10 | `pos_cov_uu`  | Float (5)  | m<sup>2</sup> | `0.072696`     | Position covariance in ENU, element UU                                     |
|   11 | `pos_cov_en`  | Float (5)  | m<sup>2</sup> | `-0.080012`    | Position covariance in ENU, element EN                                     |
|   12 | `pos_cov_nu`  | Float (5)  | m<sup>2</sup> | `0.0067274`    | Position covariance in ENU, element NU                                     |
|   13 | `pos_cov_eu`  | Float (5)  | m<sup>2</sup> | `-0.011602`    | Position covariance in ENU, element EU                                     |

### RAWIMU message

This message contains time, acceleration and angular velocity (raw value, no bias correction, only
coordinate transformation applied) in the vrtk frame - the X on the sensor. See also
[[coordinates#vrtk-output-coordinate-system]]. It is output at 200Hz IMU frequency regardless
whether fusion is running or not.

Example message:

    $FP,RAWIMU,1,2197,126191.777855,-0.199914,0.472851,9.917973,0.023436,0.007723,0.002131*34\r\n

|    # | Field         | Format     | Unit            | Example         | Description                                                          |
| ---: | ------------- | ---------- | --------------- | --------------- | -------------------------------------------------------------------- |
|    1 | `msg_type`    | String     | -               | `RAWIMU`        | Message type, always `RAWIMU` for this message                       |
|    2 | `msg_version` | Numeric    | -               | `1`             | Message version, always `1` for this version of the `RAWIMU` message |
|    3 | `gps_week`    | Numeric    | -               | `2197`          | GPS week number, range 0--9999                                       |
|    4 | `gps_tow`     | Float (.6) | s               | `126191.777855` | GPS time of week, range 0.000--604799.999                            |
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

-  The output frame of the IMU messages is the X on VRTK sensor, NOT the frame configured from the
   webinterface (They are of course the same when on webinterface the configs are 0s).

### TF message

This message contains information for static coordinate transformations.

Example message (wrapped on multiple lines for readability):

    $FP,TF,1,VRTK,CAM,0.01795,0.00044,-0.01103,
    0.485049,-0.508955,0.511098,-0.494440*4A

Message fields:

|    # | Field           | Format     | Unit | Example     | Description                                                            |
| ---: | --------------- | ---------- | ---- | ----------- | ---------------------------------------------------------------------- |
|    1 | `msg_type`      | String     | -    | `TF`        | Message type, always `ODOMETRY` for this message                       |
|    2 | `msg_version`   | Numeric    | -    | `1`         | Message version, always `1` for this version of the `ODOMETRY` message |
|    3 | `from_frame`    | Numeric    | -    | `CAM`       | GPS week number, range 0--9999                                         |
|    4 | `to_frame`      | Float (.6) | s    | `VRTK`      | GPS time of week, range 0.000--604799.999                              |
|    5 | `translation_x` | Float (.5) | m    | `0.01795`   | Position in ECEF, X component                                          |
|    6 | `translation_y` | Float (.5) | m    | `0.00044`   | Position in ECEF, Y component                                          |
|    7 | `translation_z` | Float (.5) | m    | `-0.01103`  | Position in ECEF, Z component                                          |
|    8 | `orientation_w` | Float (.6) | -    | `0.485049`  | Quaternion with respect to ECEF, W component                           |
|    9 | `orientation_x` | Float (.6) | -    | `-0.508955` | Quaternion with respect to ECEF, X component                           |
|   10 | `orientation_y` | Float (.6) | -    | `0.511098`  | Quaternion with respect to ECEF, Y component                           |
|   11 | `orientation_z` | Float (.6) | -    | `-0.494440` | Quaternion with respect to ECEF, Z component                           |

# Fixposition Odometry Converter

An extra node is provided to help with the integration of the wheel odometry on your vehicle. This node is intended to be used as a middleware if you already have a topic with the wheel odometry values running on your system. At the moment, messages of the type `Twist`, `TwistWithCov` and `Odometry` are accepted. The x component of the velocity in the input messages is then extracted, converted, and republished to the `/fixposition/speed` topic, where they will be consumed by the VRTK2. If the param `use_angular` is selected, the z component of the angular velocity (yaw rate) of the input messages is also extracted and placed in the speed vector.

_Please note that currently, the odometry converter only works for situations where the desired input odometry has just one or two values, i.e. the total vehicle speed or the total vehicle speed and yaw rate. It also assumes that the x axis of the odometry output and the VRTK2 axis are aligned. For situations where all 4 inputs are desired, a custom converter is necessary._

## Input Parameters

The `odom_converter.yaml` file exposes the necessary parameters for the correct operation of the node. The parameter that may cause the most doubt is the `multiplicative_factor`. This should be chosen such that the inputed float velocity value is transformed into milimeters per second, e.g. 1000 for an input that is expressed in meters per second.

## Launch

After the configuration is set, to launch the node simply run:

   `roslaunch fixposition_odometry_converter odom_converter.launch`

# License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
