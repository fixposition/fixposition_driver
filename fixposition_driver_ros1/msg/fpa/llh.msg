####################################################################################################
#
#    Copyright (c) 2023  ___     ___
#                       \\  \\  /  /
#                        \\  \\/  /
#                         /  /\\  \\
#                        /__/  \\__\\  Fixposition AG
#
####################################################################################################
#
# Fixposition FP_A-LLH Message
#
#
####################################################################################################

Header header
string pose_frame                # frame of the pose values [pose, quaternion]
geometry_msgs/Vector3 position   # LLH position (latitude, longitude, height)
float64[36] covariance           # Position covariance in ENU
