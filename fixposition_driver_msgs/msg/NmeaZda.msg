# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA-Gx-ZDA data

int8     talker                   # Talker, see consts.TALKER_ID_...

bool     date_valid               # Date is valid
int16    date_y                   # Date years
int8     date_m                   # Date months
int8     date_d                   # Date days

bool     time_valid               # Time is valid
int8     time_h                   # Time hours
int8     time_m                   # Time minutes
float32  time_s                   # Time seconds

int8     local_hr                 # Local zone hours, always 00 (= UTC)
int8     local_min                # Local zone minutes, always 00 (= UTC)

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
