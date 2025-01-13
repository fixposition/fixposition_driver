# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA-Gx-GST data

int8     talker                   # Talker, see consts.TALKER_ID_...

bool     time_valid               # Time is valid
int8     time_h                   # Time hours
int8     time_m                   # Time minutes
float32  time_s                   # Time seconds

float32  rms_range                # RMS value of the standard deviation of the range inputs to the navigation process, NaN if not available
float32  std_major                # Standard deviation of semi-major axis of error ellipse, NaN if not available
float32  std_minor                # Standard deviation of semi-minor axis of error ellipse, NaN if not available
float32  angle_major              # Angle of semi-major axis of error ellipse from true North, NaN if not available
float32  std_lat                  # Standard deviation of latitude error, NaN if not available
float32  std_lon                  # Standard deviation of longitude error, NaN if not available
float32  std_alt                  # Standard deviation of altitude error, NaN if not available

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
