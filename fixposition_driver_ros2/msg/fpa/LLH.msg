####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition FP_A-LLH Message
#
#
####################################################################################################

std_msgs/Header header
string pose_frame                # frame of the pose values [pose, quaternion]
geometry_msgs/Vector3 position   # LLH position (latitude, longitude, height)
float64[36] covariance           # Position covariance in ENU
