####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition FP_A-IMUBIAS Message
#
#
####################################################################################################

std_msgs/Header header
int16 fusion_imu                     # Fusion measurement status: IMU (see below)
int16 imu_status                     # IMU bias status (see below)
int16 imu_noise                      # IMU variance status (see below)
int16 imu_conv                       # IMU convergence status (see below)
geometry_msgs/Vector3 bias_acc       # Accelerometer bias
geometry_msgs/Vector3 bias_gyr       # Gyroscope bias
geometry_msgs/Vector3 bias_cov_acc   # Accelerometer bias covariance
geometry_msgs/Vector3 bias_cov_gyr   # Gyroscope bias covariance


# Fusion measurement status (fusion_imu)
#
# | Value | Description        |
# |-------|--------------------|
# |  null | Info not available |
# |   0   | Not used           |
# |   1   | Used               |
# |   2   | Degraded           |


# IMU bias status (imu_status)
#
# | Value | Description        |
# |-------|--------------------|
# |  null | Info not available |
# |   0   | Not converged      |
# |   1   | Warmstarted        |
# |   2   | Rough convergence  |
# |   3   | Fine convergence   |


# IMU variance (imu_noise)
#
# | Value | Description        |
# |-------|--------------------|
# |  null | Info not available |
# |   0   | Reserved           |
# |   1   | Low noise          |
# |   2   | Medium noise       |
# |   3   | High noise         |
# | 4...7 | Reserved           |


# IMU accelerometer and gyroscope convergence (imu_conv)
#
# | Value | Description                      |
# |-------|----------------------------------|
# |  null | Info not available               |
# |   0   | Awaiting Fusion                  |
# |   1   | Awaiting IMU measurements        |
# |   2   | Insufficient global measurements |
# |   3   | Insufficient motion              |
# |   4   | Converging                       |
# | 5...6 | Reserved                         |
# |   7   | Idle                             |