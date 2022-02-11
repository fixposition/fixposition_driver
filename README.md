# Fixposition Driver

- **Version:** 2.1.1
- **Dependencies:** Boost 1.65 or higher
- **Tested on:** Ubuntu 18.04 with ROS Melodic

This drvier operates as a ROS node, connecting to either a TCP or serial stream of Fixposition Vision-RTK output data.



## Installation
To install the node, extract this code to your catkin workspace's `src` folder and build it with:

`catkin build fixposition_driver`

Then source your development environment:

`source devel/setup.bash`

## Launch the Driver
- To launch the node in serial mode, run:

    `roslaunch fixposition_driver serial.launch`

- In TCP mode (Wi-Fi):

    `roslaunch fixposition_driver tcp.launch`

- In TCP mode (Ethernet):

    `roslaunch fixposition_driver tcp.launch`

To change the settings of TCP (IP, Port) or Serial (Baudrate, Port) connections, check the `launch/tcp.yaml` and `launch/serial.yaml`.

The output is published on four topics:

```
/fixposition/imu (type: sensor_msgs/Imu)
/fixposition/navsatfix (type: sensor_msgs/NavSatFix)
/fixposition/odometry (type: nav_msgs/Odometry)
/fixposition/vrtk (type: fixposition_driver/VRTK)

```

## Supported Data
It currently only supports parsing of input messages in the following format:
### FP:
    - `$FP,GPS_WEEK,GPS_SEC,x,y,z,w_quat,x_quat,y_quat,z_quat,x_lin_vel,y_lin_vel,z_lin_vel,x_ang_vel,y_ang_vel,z_ang_vel,x_acc,y_acc,z_acc,Flag1,Flag2,GNSS_Status,pos_cov_XX,YY,ZZ,XY,YZ,XZ,ang_cov_XX,YY,ZZ,XY,YZ,XZ,vel_cov,XX,YY,ZZ,XY,YZ,XZ,VERSION*CHECKSUM`
    - It will generate `nav_msgs::Odometry`, `sensor_msgs::Imu` and `fixpositon_output::VRTK` messages

### LLH:
    - `$LLH,GPS_WEEK,GPS_SEC,lat,lon,alt,pos_cov_XX,YY,ZZ,XY,YZ,XZ*CHECKSUM`
    - It will generate  sensor_msgs::NavSatFix messages


## Understanding the data
### Coordinate Frames
- In `/fixposition/odometry` and `/fixposition/vrtk`, the position and orientation are given wrt. **ECEF** frame. The velocity and orientation are given in the **POI** frame, which is the output coordinate frame you can configure on the VRTK sensor's output settings. By default it is on the X Fixposition logo on the housing, with X looking forward in the camera's direction, Y to the left, Z up.
- In `/fixposition/navsatfix` the position is given in Latitude [deg], Longitude [deg], Height [m].

### Status Flags
In `/fixposition/vrtk` there are status flags, they can be interpreted as follows:

| Flag 1 | Vision-Fusion Status |
| ------ | ------ |
| 0 | Not started |
| 1 | Reserved |
| 2 | Reserved |
| 3 | Inertial-GNSS initialized |
| 4 | Visual-Inertial-GNSS fusion active and running |

| Flag 2 | IMU Bias Status |
| ------ | ------ |
| 0 | Not converged |
| 1 | Converged |

| GNSS_Status | GNSS |
| ------ | ------ |
| 0 | Unknown fix |
| 1 | No fix |
| 2 | Dead-reckoning only fix |
| 3 | Time only fix |
| 4 | 2D fix |
| 5 | 3D fix |
| 6 | 3D + dead-reckoning fix |
| 7 | RTK float fix |
| 8 | RTK fixed fix |


If fusion_status <=3 then the output will be based on pure GNSS, otherwise it relies on fused GNSS + other sensor measurements.

## Code Documentation
Run `doxygen Doxyfile` to generate Doxygen code documentation.
