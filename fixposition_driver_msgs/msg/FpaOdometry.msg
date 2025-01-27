# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# FP_A-ODOMETRY data

std_msgs/Header header
string pose_frame                            # frame of the pose values (pose, quaternion)
string kin_frame                             # frame of the kinematic values (linear and angular velocity, acceleration)
geometry_msgs/PoseWithCovariance pose        # position, orientation
geometry_msgs/TwistWithCovariance velocity   # linear, angular
geometry_msgs/Vector3 acceleration           # linear acceleration

int8 fusion_status                           # Fusion status, see consts.FUSION_STATUS_LEGACY_...
int8 imu_bias_status                         # IMU bias status, see consts.IMU_STATUS_LEGACY_...
int8 gnss1_status                            # GNSS1 status, see consts.GNSS_FIX_...
int8 gnss2_status                            # GNSS2 status, see consts.GNSS_FIX_...
int8 wheelspeed_status                       # Wheelspeed status, see consts.WS_STATUS_LEGACY_...

string version                               # Fixposition software version

fixposition_driver_msgs/FpaConsts consts # Constants for enums used in FP_A
