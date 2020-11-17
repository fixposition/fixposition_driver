# FP Message Converter

**Version:** 1.0.0

**Dependencies:** Boost 1.66 or higher

**Tested on: ** Ubuntu 18.04, ROS Melodic

This converter operates as a ROS node, connecting to either a TCP or serial stream of Fixposition output data and publishing the state messages as both nav_msgs::Odometry and fixpositon_output::VRTK messages.

It currently only supports parsing of input messages in the following format:

`$FP,GPS_WEEK,GPS_SEC,x,y,z,x_vel,y_vel,z_vel,w_quat,x_quat,y_quat,z_quat,STATUS,VERSION*CHK`

To install the node, extract this code to your catkin workspace's `src` folder and build it with:

`catkin build`

To launch the node (in serial mode, for instance), run:

`roslaunch fixposition_output fp_output.launch input_type:=serial input_port:=/dev/ttyUSB0 serial_baudrate:=115200 pub_rate:=200`

Or, in TCP mode:

`roslaunch fixposition_output fp_output.launch input_type:=tcp input_port:=21000 tcp_ip:=192.168.49.1 pub_rate:=200`

All arguments are optional and have the default values specified in the previous examples. If no args are given, the node runs in TCP mode by default.

The output is published on two topics:

```
/fixposition/odometry
/fixposition/vrtk
```
For each message type, respectively. The VRTK message is a custom format that includes the odometry information plus two extra fields for the current Visions-Fusion and GNSS status flags:
```
uint16 fusion_status                        
uint16 gnss_status
```
These can be interpreted as follows:

| fusion_status | Vision-Fusion Status |
| ------ | ------ |
| 0 | Not Started |
| 1 | Diverged |
| 2 | Initializing |
| 4 | "Dead-Reckoning" (no GNSS measurement taken) | 
| 5 | Initialized, pipeline running and giving output |


| gnss_status | GNSS |
| ------ | ------ |
| 0 | N/A |
| 1 | GNSS Single |
| 2 | GNSS Float |
| 3 | GNSS Fix |

For example:
- **00** = nothing is running
- **53** = best quality
- **52** = Fusion quality is very good, GNSS is in Float
- **42** = Fusion quality is very good, GNSS in Float, but rejected due to high covariance
- **13** = GNSS is in Fix, but Fusion has diverged and is resetting

If fusion_status <=3 then the output will be based on pure GNSS, otherwise it relies on fused GNSS + other sensor measurements.
