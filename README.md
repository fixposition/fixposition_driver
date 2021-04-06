# FP Message Converter

**Version:** 1.0.0

**Dependencies:** Boost 1.65 or higher

**Tested on:** Ubuntu 18.04 with ROS Melodic

This converter operates as a ROS node, connecting to either a TCP or serial stream of Fixposition output data and publishing the state messages as both nav_msgs::Odometry and fixpositon_output::VRTK messages.

It currently only supports parsing of input messages in the following format:

`$FP,GPS_WEEK,GPS_SEC,x,y,z,x_vel,y_vel,z_vel,w_quat,x_quat,y_quat,z_quat,STATUS,VERSION*CHK`

To install the node, extract this code to your catkin workspace's `src` folder and build it with:

`catkin build`

Then source your development environment:

`source devel/setup.bash`

To launch the node (in serial mode, for instance), run:

`roslaunch fixposition_output fp_output.launch input_type:=serial input_port:=/dev/ttyUSB0 serial_baudrate:=115200 pub_rate:=200`

Or, in TCP mode:

`roslaunch fixposition_output fp_output.launch input_type:=tcp input_port:=21000 tcp_ip:=192.168.49.1 pub_rate:=200`

All arguments are optional and have the default values specified in the previous examples. If no args are given, the node runs in TCP mode by default.

The output is published on two topics:

```
/fixposition/imu
/fixposition/navsatfix
/fixposition/odometry
/fixposition/vrtk

```
For each message type, respectively. The VRTK message is a custom format that includes the odometry information, plus IMU acceleration and the current Visions-Fusion and GNSS status flags:
```
Header header
string pose_frame                               # frame of the pose values [pose, quaternion]
string kin_frame                                # frame of the kinematic values [linear/angular velocity, acceleration]
geometry_msgs/PoseWithCovariance pose           # position, orientation
geometry_msgs/TwistWithCovariance velocity      # linear, angular
geometry_msgs/Vector3 acceleration              # linear acceleration

uint16 fusion_status                            # field for the fusion status
uint16 imu_bias_status                          # field for the IMU bias status
uint16 gnss_status                              # field for the gnss status
string version                                  # Fixposition software version


```
These can be interpreted as follows:

| fusion_status | Vision-Fusion Status |
| ------ | ------ |
| 0 | Not started, outputting position only based on pure gnss.
| 1 | Reset triggered
| 2 | Initializing
| 3 | Single-side gnss initialization
| 4 | Dead reckoning
| 5 | GNSS fused, active

| imu_bias_status | IMU Bias Status |
| ------ | ------ |
| 0 | Not converged |
| 1 | Converged |

| gnss_status | GNSS |
| ------ | ------ |
| 0 | N/A |
| 1 | GNSS Single |
| 2 | GNSS Float |
| 3 | GNSS Fix |


If fusion_status <=3 then the output will be based on pure GNSS, otherwise it relies on fused GNSS + other sensor measurements.
