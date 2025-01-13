# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA epoch data collected from all NMEA messages

std_msgs/Header header            # Header, timestamp non-zero if date_valid and time_valid

bool     date_valid               # Date is valid
int16    date_y                   # Date years
int8     date_m                   # Date months
int8     date_d                   # Date days

bool     time_valid               # Time is valid
int8     time_h                   # Time hours
int8     time_m                   # Time minutes
float32  time_s                   # Time seconds

int8     status                   # Positioning system status, see consts.STATUS_GLL_RMC_...
int8     navstatus                # Navigational status (optional), see consts.NAV_STATUS_RMC_...
int8     mode1                    # Positioning system mode indicator, see consts.MODE_RMC_GNS_...
int8     mode2                    # Positioning system mode, see consts.MODE_GLL_VTG_...
int8     quality                  # Fix quality, see consts.QUALITY_GGA_...
int8     opmode                   # Operation mode, see consts.OP_MODE_GSA_...
int8     navmode                  # Nav mode, see consts.NAV_MODE_GSA_...

float64  latitude                 # Latitude [deg], NaN if not available
float64  longitude                # Longitude [deg], NaN if not available
float64  height                   # Height [m], NaN if not available

int8     num_sv                   # Number of satellites used in solution, -1 if not available

float32  rms_range                # RMS value of the standard deviation of the range inputs to the navigation process, NaN if not available
float32  std_major                # Standard deviation of semi-major axis of error ellipse, NaN if not available
float32  std_minor                # Standard deviation of semi-minor axis of error ellipse, NaN if not available
float32  angle_major              # Angle of semi-major axis of error ellipse from true North, NaN if not available
float32  std_lat                  # Standard deviation of latitude error, NaN if not available
float32  std_lon                  # Standard deviation of longitude error, NaN if not available
float32  std_alt                  # Standard deviation of altitude error, NaN if not available

float32  pdop                     # PDOP, NaN if not available
float32  hdop                     # PDOP, NaN if not available
float32  vdop                     # PDOP, NaN if not available

float32  heading                  # True heading [deg], NaN if not available
float32  speed                    # Speed over ground [knots], NaN if not available
float32  course                   # Course over ground w.r.t. True North [deg], NaN if not available

float32  cogt                     # Course over ground (true) [deg], NaN if not available
float32  cogm                     # Course over ground (magnetic) [deg], NaN if not available
float32  sogn                     # Speed over ground [knots], NaN if not available
float32  sogk                     # Speed over ground [km/h], NaN if not available

float32  diff_age                 # Differential data age, NaN if not available
int16    diff_sta                 # Differential station ID

int8     local_hr                 # Local zone hours, always 00 (= UTC)
int8     local_min                # Local zone minutes, always 00 (= UTC)

fixposition_driver_msgs/NmeaSatellite[] sats  # Satellites
fixposition_driver_msgs/NmeaSignal[]    sigs  # Signals

fixposition_driver_msgs/NmeaConsts consts # Constants for enums used in NMEA

float64[9] cov_enu  # ENU position covariance, based on std_{lat,lon,alt} or {pdop,hdop}, all 0 if not available
