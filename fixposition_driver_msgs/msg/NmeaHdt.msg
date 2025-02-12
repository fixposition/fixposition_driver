# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA-Gx-HDT data

int8     talker                   # Talker, see consts.TALKER_ID_...

float32  heading                  # True heading [deg], NaN if not available

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
