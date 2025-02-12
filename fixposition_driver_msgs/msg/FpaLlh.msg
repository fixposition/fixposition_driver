# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# FP_A-LLH data

std_msgs/Header header
geometry_msgs/Vector3 position   # LLH position (latitude, longitude, height)
float64[9] covariance            # Position covariance in ENU

fixposition_driver_msgs/FpaConsts consts # Constants for enums used in FP_A
