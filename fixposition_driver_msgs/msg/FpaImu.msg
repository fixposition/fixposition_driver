# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# FP_A-RAWIMU or FP_A-CORRIMU data

bool bias_comp                               # Signal is bias compensated (true) or not (false), always false for RAWIMU, may be true for CORRIMU
int8 imu_status                              # IMU bias status, see consts.IMU_STATUS_...
sensor_msgs/Imu data                         # IMU data

# Note that bias_comp and imu_status are available since version 2.119.0 of the Vision-RTK 2 software.

fixposition_driver_msgs/FpaConsts consts # Constants for enums used in FP_A
