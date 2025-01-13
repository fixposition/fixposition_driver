# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# FP_A-TP data

string  tp_name                              # Timepulse name (source)
string  timebase                             # Time base, see consts.TIMEBASE_...
string  timeref                              # Time reference, see consts.TIMEREF_...
int16   tp_week                              # Timepulse week (only available in FP_A-TP version 2)
int64   tp_tow_sec                           # Timepulse time seconds of week, integer second part (0–604799), or null
float64 tp_tow_psec                          # Timepulse time seconds of week, sub-second part (0.000000000000–0.999999999999), or null
int64   gps_leaps                            # GPS leapseconds, or null if unknown

fixposition_driver_msgs/FpaConsts consts # Constants for enums used in FP_A
