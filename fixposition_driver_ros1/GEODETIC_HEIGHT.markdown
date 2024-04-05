## Calculation Altitude in Mining Industry

Reference:
```
https://docs.ros.org/en/kinetic/api/gtsam/html/classNETGeographicLib_1_1Geoid.html

```
Online tool to calculate the geodetic height (it is different from height above the geoid)

```
https://geographiclib.sourceforge.io/cgi-bin/GeodSolve

https://support.virtual-surveyor.com/support/solutions/articles/1000261349-the-difference-between-ellipsoidal-geoid-and-orthometric-elevations-
```

To calculate the height above the geoid (normal altitude used in the mining industry), use the following equation:

```bash
Height above the Geoid = Height above the Ellipsoid - Geodetic Height
```

Where:
- **Height above the Geoid** is the normal altitude used in the mining industry. Also called orthometric height 
- **Height above the Ellipsoid** is the normal height in Fixposition's output. Also called ellipsoidal height
- **Geodetic Height** is the height above the geoid model.

To achieve this, leveraging GeographicLib is a good option. Below are the steps to consider for enabling this in the Fixposition driver ROS1 (Enable ROS2 is in the todo list):

## Git Clone the Codes from the Specific Branch

Clone the Repository: Use the git clone command with the -b option to specify the branch `geodetic_height/egm2008` branch from the Jelvon GitHub repository.

```
git clone -b geodetic_height/egm2008 https://github.com/Jelvon/fixposition_driver.git
```
Then we are going to integrate GeographicLib with Fixposition driver ros 1

## Prerequisites

Before proceeding with the installation of GeographicLib, ensure your system meets the following requirements:
- Ubuntu 18.04 or newer
- ROS1
- C++ Compiler with C++11 support

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

## Installation Instructions

### GeographicLib Installation

Install GeographicLib and its development files:

```sh
sudo apt-get update
sudo apt-get install libgeographic-dev
```

### GeographicLib Tools

Install the `geographiclib-tools` package, which includes utilities for managing geoid model data:

```sh
sudo apt install geographiclib-tools
```

### Downloading Geoid Model Data

Download the EGM2008 geoid model data. Note that this might take some time due to the size of the data files:

```sh
sudo geographiclib-get-geoids egm2008-1
```

**Note**: It's recommended to perform this step on your host PC rather than embedding it directly into VRTK2's firmware, due to the substantial size of the geoid model data files.

### Verification

Confirm that the geoid models have been correctly installed:

```sh
ls -l /usr/share/GeographicLib/geoids/
```

## Configuration in CMakeLists.txt

Incorporate GeographicLib into your ROS1 package by adjusting your `CMakeLists.txt`:

```cmake
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)
```
This modification ensures CMake locates the GeographicLib configuration module, enabling its functionalities within your project.

After installing `libgeographic-dev`, the `FindGeographicLib.cmake` file is typically located in a directory where CMake can find it. However, the exact path (e.g. `/usr/share/cmake/geographiclib`) can vary based on the system configuration and how GeographicLib was installed.

you can use `dpkg-query` to list the files installed by the `libgeographic-dev` package:

```
dpkg-query -L libgeographic-dev
```

Look for paths that contain cmake or geographiclib in the output. These paths are likely where CMake configuration files are located. Then you can use this path to modify the `CMakeLists.txt`.

## Installation

To install the node, arrange the folders like this: (don't forget to git clone the `fixposition_gnss_tf` from branch `main`)

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
source /opt/ros/{ROS_DISTRO}/setup.bash
```

and build it with:

`catkin build fixposition_driver_ros1`

This will build the ROS1 driver node and all its dependencies.

Then source your development environment:

`source devel/setup.bash`

## Launch the Driver

-  Enable the output for "LLH_MINING" in the yaml file to obtain specialized height-above-geoid information, This setting will be utilized in future operations to access specific geodetic data:

   ` formats: ["ODOMETRY", "LLH", "LLH_MINING", "RAWIMU", "CORRIMU", "GPGGA", "GPZDA", "GPRMC", "TF"] `

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
   /fixposition/navsatfix_mining
   ```

- Check published message in the topic, for example the VRTK message:
   `rostopic echo /fixposition/mining`

Now the height inside the topic is the height above the geoid (normal altitude used in the mining industry)




