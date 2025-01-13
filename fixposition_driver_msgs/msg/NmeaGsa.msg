# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA-Gx-GSA data

int8     talker                   # Talker, see consts.TALKER_ID_...
int8     system                   # System, see consts.SYSTEM_ID_...

int8[]   sats                     # Satellites

int8     opmode                   # Operation mode, see consts.OP_MODE_GSA_...
int8     navmode                  # Nav mode, see consts.NAV_MODE_GSA_...

float32  pdop                     # PDOP, NaN if not available
float32  hdop                     # PDOP, NaN if not available
float32  vdop                     # PDOP, NaN if not available

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
