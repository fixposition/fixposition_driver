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
```


## Installation

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
make sure you have sourced the setup.bash from ros:

`/opt/ros/{ROS_DISTRO}/setup.bash`, for example

```
source /opt/ros/foxy/setup.bash`
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
   /fixposition/odometry_enu
   /fixposition/poiimu
   /fixposition/rawimu
   /fixposition/vrtk
   ```

- Check published message in the topic, for example the VRTK message:
   `ros2 topic echo /fixposition/vrtk`
