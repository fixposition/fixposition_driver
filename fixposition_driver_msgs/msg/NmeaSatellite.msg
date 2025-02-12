# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA satellite info

int8     system                   # System, see consts.SYSTEM_ID_...
int8     svid                     # Satellite
int16    az                       # Azimuth [deg]
int16    el                       # Elevations [deg]

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
