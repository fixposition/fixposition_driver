# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA-Gx-RMC data

int8     talker                   # Talker, see consts.TALKER_ID_...

bool     date_valid               # Date is valid
int16    date_y                   # Date years
int8     date_m                   # Date months
int8     date_d                   # Date days

bool     time_valid               # Time is valid
int8     time_h                   # Time hours
int8     time_m                   # Time minutes
float32  time_s                   # Time seconds

int8     status                   # Positioning system status, see consts.STATUS_GLL_RMC_...
int8     mode                     # Positioning system mode indicator, see consts.MODE_RMC_GNS_...
int8     navstatus                # Navigational status (optional), see consts.NAV_STATUS_RMC_...

float64  latitude                 # Latitude [deg], NaN if not available
float64  longitude                # Longitude [deg], NaN if not available
float64  height                   # Height [m], NaN if not available

float32  speed                    # Speed over ground [knots], NaN if not available
float32  course                   # Course over ground w.r.t. True North [deg], NaN if not available

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA
