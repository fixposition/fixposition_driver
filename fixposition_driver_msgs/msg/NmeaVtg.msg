# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA-Gx-VTG data

int8     talker                   # Talker, see consts.TALKER_ID_...

float32  cogt                     # Course over ground (true) [deg], NaN if not available
float32  cogm                     # Course over ground (magnetic) [deg], NaN if not available
float32  sogn                     # Speed over ground [knots], NaN if not available
float32  sogk                     # Speed over ground [km/h], NaN if not available

int8     mode                     # Positioning system mode, see consts.MODE_GLL_VTG_...

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
