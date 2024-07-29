####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition FP_A-ODOMENU Message
#
#
####################################################################################################

std_msgs/Header header
string pose_frame                            # frame of the pose values [pose, quaternion]
string kin_frame                             # frame of the kinematic values [linear/angular velocity, acceleration]
geometry_msgs/PoseWithCovariance pose        # position, orientation
geometry_msgs/TwistWithCovariance velocity   # linear, angular
geometry_msgs/Vector3 acceleration           # linear acceleration

int16 fusion_status                          # field for the fusion status
int16 imu_bias_status                        # field for the IMU bias status
int16 gnss1_status                           # field for the gnss1 status
int16 gnss2_status                           # field for the gnss2 status
int16 wheelspeed_status                      # field for the wheelspeed status
