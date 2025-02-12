# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA-Gx-GLL data

int8     talker                   # Talker, see consts.TALKER_ID_...

bool     time_valid               # Time is valid
int8     time_h                   # Time hours
int8     time_m                   # Time minutes
float32  time_s                   # Time seconds

float64  latitude                 # Latitude [deg], NaN if not available
float64  longitude                # Longitude [deg], NaN if not available

int8     status                   # Positioning system status, see consts.STATUS_GLL_RMC_...
int8     mode                     # Positioning system mode, see consts.MODE_GLL_VTG_...

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
