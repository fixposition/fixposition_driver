# Fixposition Driver ROS1

[ROS](https://www.ros.org/) (both ROS1 and ROS2) Driver for [Fixposition Vision-RTK 2](https://www.fixposition.com/product)

## Dependencies

-  [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), tested with version [3.3.7](https://gitlab.com/libeigen/eigen/-/releases/3.3.7)
-  [Boost](https://www.boost.org/), tested with version [1.65.0](https://www.boost.org/users/history/version_1_65_0.html)
-  [CMake](https://cmake.org/)
-  [tf](http://wiki.ros.org/tf) ROS1 library
-  [eigen_conversions](https://wiki.ros.org/eigen_conversions) ROS1 library
-  [Catkin](http://wiki.ros.org/catkin) for ROS1

-  **[fixposition_gnss_tf](https://github.com/fixposition/fixposition_gnss_tf)**: Fixposition GNSS Transformation Lib


This driver operates as a ROS node, connecting to either a TCP or serial stream of Fixposition Vision-RTK output data, see [Fixposition ASCII messages](#fixposition-ascii-messages) and the **Integration Manual**.

## Installing dependencies on Ubuntu system

```
 sudo apt update
 sudo apt install -y build-essential cmake
 sudo apt install -y libeigen3-dev
 sudo apt install -y ros-{ROS_DISTRO}-tf ros-{ROS_DISTRO}-eigen-conversions
```


## Installation

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
make sure you have sourced the setup.bash from ros:

`/opt/ros/{ROS_DISTRO}/setup.bash`, for example

```
source /opt/ros/melodic/setup.bash
```

and build it with:

`catkin build fixposition_driver_ros1`

This will build the ROS1 driver node and all its dependencies.

Then source your development environment:

`source devel/setup.bash`

## Launch the Driver

-  To launch the node in serial mode, run:

   `roslaunch fixposition_driver_ros1 serial.launch`

-  In TCP mode (Wi-Fi):

   `roslaunch fixposition_driver_ros1 tcp.launch`

-  In TCP mode (Ethernet):

   `roslaunch fixposition_driver_ros1 tcp.launch`

To change the settings of TCP (IP, Port) or Serial (Baudrate, Port) connections, check the `launch/tcp.yaml` and `launch/serial.yaml` files.

## Check the messages
- Check rostopics:

  `rostopic list`


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
   `rostopic echo /fixposition/vrtk`
