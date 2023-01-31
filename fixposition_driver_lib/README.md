# Fixposition Driver Lib

This is a CMake library used to parse [Fixposition ASCII messages](../README.md#fixposition-ascii-messages). The message content will be converted into a generic struct and can be processed further from there.

It can be built using plain CMake, or using catkin or colcon depending on which ROS version is used.

## Dependencies
-  [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page), tested with version [3.3.7](https://gitlab.com/libeigen/eigen/-/releases/3.3.7)
-  [Boost](https://www.boost.org/), tested with version [1.65.0](https://www.boost.org/users/history/version_1_65_0.html)
-  [CMake](https://cmake.org/)

ROS1:
-  [Catkin](http://wiki.ros.org/catkin)

ROS2
-  [Colcon](https://colcon.readthedocs.io/en/released/)


## Build
### CMake
from the repository root dir:
```
cd fixposition_driver_lib
mkdir build
cd build
cmake ..
make
```

### Catkin
clone this repository under the `src` folder in the catkin workspace, and then run `catkin build fixposition_driver_lib` or if you want to build all packages, run `catkin build`.

### Colcon
clone this repository under the `src` folder in the colcon workspace, and then run `colcon build --packages-up-to fixposition_driver_lib` or if you want to build all packages, run `colcon build`

### Use this library in other projects:
A cmake file is provided in the cmake directory, please see [fixposition_driver_ros1](../fixposition_driver_ros1) or [fixposition_driver_ros2](../fixposition_driver_ros2) as examples of using this library.
