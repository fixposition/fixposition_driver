# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA-Gx-GSV data

int8     talker                   # Talker, see consts.TALKER_ID_...
int8     system                   # System, see consts.SYSTEM_ID_...
int8     signal                   # Signal, see consts.SIGNAL_ID_...

int8     num_msgs                 # Number of messages in this GSV sequence (for this system and signal)
int8     msg_num                  # Message number in sequence (1...num_msgs)

int8[]   azel_sat                 # Satellite for azimuth/elevation
int16[]  el                       # Elevations [deg]
int16[]  az                       # Azimuth [deg]

int8[]   cno_sat                  # Satellite for signal level
int16[] cno                       # Signal level [dbHz]

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
