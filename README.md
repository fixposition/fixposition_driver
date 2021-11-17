# FP Message Converter

**Version:** 2.0.0

**Dependencies:** Boost 1.65 or higher

**Tested on:** Ubuntu 18.04 with ROS Melodic

This converter operates as a ROS node, connecting to either a TCP or serial stream of Fixposition output data and publishing the state messages as both nav_msgs::Odometry and fixpositon_output::VRTK messages.

It currently only supports parsing of input messages in the following format:

`$FP,GPS_WEEK,GPS_SEC,x,y,z,w_quat,x_quat,y_quat,z_quat,x_lin_vel,y_lin_vel,z_lin_vel,x_ang_vel,y_ang_vel,z_ang_vel,x_acc,y_acc,z_acc,Flag1,Flag2,GNSS_Status,pos_cov_XX_YY_ZZ_XY_YZ_XZ,ang_cov_XX_YY_ZZ_XY_YZ_XZ,vel_cov_XX_YY_ZZ_XY_YZ_XZ,VERSION*CHK`

To install the node, extract this code to your catkin workspace's `src` folder and build it with:

`catkin build`

Then source your development environment:

`source devel/setup.bash`

To launch the node (in serial mode), run:

`roslaunch fixposition_output fp_output.launch input_type:=serial input_port:=/dev/ttyUSB0 serial_baudrate:=115200 pub_rate:=200`

In TCP mode (Wi-Fi):

`roslaunch fixposition_output fp_output.launch input_type:=tcp input_port:=21000 tcp_ip:=10.0.1.1 pub_rate:=200`

In TCP mode (Ethernet):

`roslaunch fixposition_output fp_output.launch input_type:=tcp input_port:=21000 tcp_ip:=10.0.2.1 pub_rate:=200`

All arguments are optional and have the default values specified in the previous examples. If no args are given, the node runs in TCP mode by default.

The output is published on four topics:

```
/fixposition/imu (type: sensor_msgs/Imu)
/fixposition/navsatfix (type: sensor_msgs/NavSatFix)
/fixposition/odometry (type: nav_msgs/Odometry)
/fixposition/vrtk (type: fixposition_output/VRTK)

```


The flags can be interpreted as follows:

| Flag 1 | Vision-Fusion Status |
| ------ | ------ |
| 0 | Not started |
| 1 | Reserved |
| 2 | Reserved |
| 3 | Reserved |
| 4 | VIO enabled |

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
