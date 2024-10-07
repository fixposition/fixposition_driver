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
# Fixposition FP_A-GNSSCORR Message
#
#
####################################################################################################

Header header
int16 gnss1_fix          # GNSS1 fix status
int16 gnss1_nsig_l1      # Number of L1 signals with correction data tracked by GNSS1
int16 gnss1_nsig_l2      # Number of L2 signals with correction data tracked by GNSS1
int16 gnss2_fix          # GNSS2 fix status
int16 gnss2_nsig_l1      # Number of L1 signals with correction data tracked by GNSS2
int16 gnss2_nsig_l2      # Number of L2 signals with correction data tracked by GNSS2

float64 corr_latency       # Average correction data latency (10s window)
float64 corr_update_rate   # Average correction update rate (10s window)
float64 corr_data_rate     # Average correction data rate (10s window)
float64 corr_msg_rate      # Average correction message rate (10s window)

int16 sta_id                    # Correction station ID, range 0–4095
geometry_msgs/Vector3 sta_llh   # Correction station LLH position (latitude, longitude, height)
int32 sta_dist                  # Correction station baseline length
