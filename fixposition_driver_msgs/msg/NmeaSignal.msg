# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA signal info

int8     system                   # System, see consts.SYSTEM_ID_...
int8     svid                     # Satellite
int8     signal                   # Signal, see consts.SIGNAL_ID_...
float32  cno                      # Signal level [dbHz]
bool     used                     # Signal used

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
