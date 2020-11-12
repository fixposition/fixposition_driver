# FP Message Converter

This converter operates as a ROS node, connecting to either a TCP or serial stream of Fixposition output data and publishing the state messages as nav_msgs::Odometry messages.

It currently only supports parsing of messages in the following format:

`$FP,GPS_WEEK,GPS_SEC,x,y,z,x_vel,y_vel,z_vel,w_quat,x_quat,y_quat,z_quat,STATUS,VERSION*CHK`

To install the node, download this code to your catkin workspace's `src` folder and build it with:

`catkin build`

 **Note** that this package depends on Boost 1.66 or higher.

To launch the node (in serial mode, for instance), run:

`roslaunch fixposition_output fp_output.launch input_type:=serial input_port:=/dev/ttyUSB0 serial_baudrate:=115200`

Or, in TCP mode:

`roslaunch fixposition_output fp_output.launch input_type:=tcp input_port:=21000 tcp_ip:=192.168.49.1`

All arguments are optional and have the default values specified in the previous examples. If no args are given, the node runs in TCP mode by default.
