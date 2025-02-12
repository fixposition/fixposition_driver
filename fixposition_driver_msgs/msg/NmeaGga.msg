# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA-Gx-GGA data

int8     talker                   # Talker, see consts.TALKER_ID_...

bool     time_valid               # Time is valid
int8     time_h                   # Time hours
int8     time_m                   # Time minutes
float32  time_s                   # Time seconds

float64  latitude                 # Latitude [deg], NaN if not available
float64  longitude                # Longitude [deg], NaN if not available
float64  height                   # Height [m], NaN if not available

int8     quality                  # Fix quality, see consts.QUALITY_GGA_...
int8     num_sv                   # Number of satellites used in solution, -1 if not available

float32  hdop                     # Horizontal dilution of precision, NaN if not available

float32  diff_age                 # Differential data age, NaN if not available
int16    diff_sta                 # Differential station ID

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
