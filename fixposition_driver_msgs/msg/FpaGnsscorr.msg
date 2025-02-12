# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# FP_A-GNSSCORR data

std_msgs/Header header
int8 gnss1_fix          # GNSS1 fix status, see consts.GNSS_FIX_...
int8 gnss1_nsig_l1      # Number of L1 signals with correction data tracked by GNSS1, -1 if n/a
int8 gnss1_nsig_l2      # Number of L2 signals with correction data tracked by GNSS1, -1 if n/a
int8 gnss2_fix          # GNSS2 fix status, see consts.GNSS_FIX_...
int8 gnss2_nsig_l1      # Number of L1 signals with correction data tracked by GNSS2, -1 if n/a
int8 gnss2_nsig_l2      # Number of L2 signals with correction data tracked by GNSS2, -1 if n/a

float32 corr_latency       # Average correction data latency (10s window)
float32 corr_update_rate   # Average correction update rate (10s window)
float32 corr_data_rate     # Average correction data rate (10s window)
float32 corr_msg_rate      # Average correction message rate (10s window)

int16 sta_id                    # Correction station ID, range 0–4095, -1 if n/a
geometry_msgs/Vector3 sta_llh   # Correction station LLH position (latitude, longitude, height)
int32 sta_dist                  # Correction station baseline length, -1 if n/a

fixposition_driver_msgs/FpaConsts consts # Constants for enums used in FP_A
