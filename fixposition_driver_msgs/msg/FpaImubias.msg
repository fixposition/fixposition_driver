# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# FP_A-IMUBIAS data

std_msgs/Header header
int16 fusion_imu                     # Fusion measurement status: IMU, see consts.MEAS_STATUS_...
int16 imu_status                     # IMU bias status, see consts.IMU_STATUS_...
int16 imu_noise                      # IMU variance status, see consts.IMU_NOISE_...
int16 imu_conv                       # IMU convergence status, see consts.IMU_CONV_...
geometry_msgs/Vector3 bias_acc       # Accelerometer bias
geometry_msgs/Vector3 bias_gyr       # Gyroscope bias
geometry_msgs/Vector3 bias_cov_acc   # Accelerometer bias covariance
geometry_msgs/Vector3 bias_cov_gyr   # Gyroscope bias covariance

fixposition_driver_msgs/FpaConsts consts # Constants for enums used in FP_A
