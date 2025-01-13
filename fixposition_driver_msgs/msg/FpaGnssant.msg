# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# FP_A-GNSSANT data

std_msgs/Header header
int8   gnss1_state   # GNSS1 antenna state, see consts.ANT_STATE_...
int8   gnss1_power   # GNSS1 antenna power, see consts.ANT_POWER_...
int32  gnss1_age     # Time since gnss1_state or gnss1_power information last changed
int8   gnss2_state   # GNSS2 antenna state, see consts.ANT_STATE_...
int8   gnss2_power   # GNSS2 antenna power, see consts.ANT_POWER_...
int32  gnss2_age     # Time since gnss2_state or gnss2_power information last changed

fixposition_driver_msgs/FpaConsts consts # Constants for enums used in FP_A
