####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition FP_A-ODOMSTATUS Message
#
#
####################################################################################################

std_msgs/Header header
int16 init_status                            # Fusion init status (see below)
int16 fusion_imu                             # Fusion measurement status: IMU (see below)
int16 fusion_gnss1                           # Fusion measurement status: GNSS 1 (see below)
int16 fusion_gnss2                           # Fusion measurement status: GNSS 2 (see below)
int16 fusion_corr                            # Fusion measurement status: GNSS corrections (see below)
int16 fusion_cam1                            # Fusion measurement status: camera (see below)
int16 fusion_ws                              # Fusion measurement status: wheelspeed (see below)
int16 fusion_markers                         # Fusion measurement status: markers (see below)
int16 imu_status                             # IMU bias status (see below)
int16 imu_noise                              # IMU variance status (see below)
int16 imu_conv                               # IMU convergence status (see below)
int16 gnss1_status                           # GNSS 1 status (see below)
int16 gnss2_status                           # GNSS 2 status (see below)
int16 baseline_status                        # Baseline status (see below)
int16 corr_status                            # GNSS correction status (see below)
int16 cam1_status                            # Camera 1 status (see below)
int16 ws_status                              # Wheelspeed status (see below)
int16 ws_conv                                # Wheelspeed convergence status (see below)
int16 markers_status                         # Markers status (see below)
int16 markers_conv                           # Markers convergence status (see below)


# Fusion initialisation status (init_status)
#
# | Value | Description          |
# |-------|----------------------|
# |  null | Unknown              |
# |   0   | Not initialized      |
# |   1   | Locally initialised  |
# |   2   | Globally initialised |


# Fusion measurement status (fusion_imu, fusion_cam1, fusion_cam2, fusion_gnss1, fusion_gnss2, fusion_corr, fusion_ws, fusion_markers)
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


# GNSS fix status (gnss1_status, gnss2_status)
#
# | Value | Description                    |
# |-------|--------------------------------|
# |  null | Info not available             |
# |   0   | No fix                         |
# |   1   | Single-point positioning (SPP) |
# |   2   | RTK moving baseline            |
# | 3...4 | Reserved                       |
# |   5   | RTK float                      |
# | 6...7 | Reserved                       |
# |   8   | RTK fixed                      |


# GNSS correction status (corr_status)
#
# | Value | Description                                                                                                    |
# |-------|----------------------------------------------------------------------------------------------------------------|
# |  null | Info not available                                                                                             |
# |   0   | Waiting fusion                                                                                                 |
# |   1   | No GNSS available                                                                                              |
# |   2   | No corrections used                                                                                            |
# |   3   | Limited corrections used: station data & at least one of the constellations among the valid rover measurements |
# |   4   | Corrections are too old                                                                                        |
# |   5   | Sufficient corrections used: station data and corrections for ALL bands among the valid rover measurements     |


# Baseline status (baseline_status)
#
# | Value | Description            |
# |-------|------------------------|
# |  null | Info not available     |
# |   0   | Waiting fusion         |
# |   1   | Not available / No fix |
# |   2   | Failing                |
# |   3   | Passing                |


# Camera status (cam1_status)
#
# | Value | Description                                      |
# |-------|--------------------------------------------------|
# |  null | Info not available                               |
# |   0   | Camera not available                             |
# |   1   | Camera available, but not usable (e.g. too dark) |
# | 2...4 | Reserved                                         |
# |   5   | Camera working and available                     |


# Wheelspeed status (ws_status)
#
# | Value | Description                                                |
# |-------|------------------------------------------------------------|
# |  null | Info not available                                         |
# |   0   | No wheelspeed enabled                                      |
# |   1   | Missing wheelspeed measurements                            |
# |   2   | At least one wheelspeed enabled, no wheelspeed converged   |
# |   3   | At least one wheelspeed enabled and at least one converged |
# |   4   | At least one wheelspeed enabled and all converged          |


# Wheelspeed convergence status (ws_conv)
#
# | Value | Description                       |
# |-------|-----------------------------------|
# |  null | Info not available                |
# |   0   | Awaiting Fusion                   |
# |   1   | Missing wheelspeed measurements   |
# |   2   | Insufficient global measurements  |
# |   3   | Insufficient motion               |
# |   4   | Insufficient imu bias convergence |
# |   5   | Converging                        |
# |   6   | Idle                              |


# Markers status (markers_status)
#
# | Value | Description                 |
# |-------|-----------------------------|
# |  null | Info not available          |
# |   0   | No markers available        |
# |   1   | Markers available           |
# |   2   | Markers available, and used |


# Markers convergence status (markers_conv)
#
# | Value | Description                      |
# |-------|----------------------------------|
# |  null | Info not available               |
# |   0   | Awaiting Fusion                  |
# |   1   | Waiting marker measurements      |
# |   2   | Insufficient global measurements |
# |   3   | Converging                       |
# |   4   | Idle                             |
