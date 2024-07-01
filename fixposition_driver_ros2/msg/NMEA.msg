####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition NMEA Message. Specified using the WGS 84 reference ellipsoid.
#
#
####################################################################################################

# Format        | Field       | Unit              | Description
# --------------|-------------|-------------------|----------------------------------------------------------------------|
std_msgs/Header   header      # [ns]              | Specifies the ROS time and Euclidian reference frame.
string            time        # [hhmmss.ss(ss)]   | UTC time (hours, minutes and seconds).
string            date        # [ddmmyy]          | UTC date (day, month and year).
float64           latitude    # [ddmm.mmmmm(mm)]  | Latitude. Positive is north of equator; negative is south.
float64           longitude   # [dddmm.mmmmm(mm)] | Longitude. Positive is east of prime meridian; negative is west.
float64           altitude    # [m]               | Altitude. Positive is above the WGS 84 ellipsoid.
int8              quality     # [-]               | Quality indicator (see below).
int8              num_sv      # [-]               | Number of satellites. Strict NMEA: 00-12. High-precision NMEA: 00-99.
int8[]            ids         # [-]               | ID numbers of satellites used in solution. See the NMEA 0183 version 4.11 standard document.
float64           hdop_rec    # [0.10-99.99]      | Horizontal dilution of precision.
float64           pdop        # [-]               | Position dillution of precision.
float64           hdop        # [-]               | Horizontal dillution of precision.
float64           vdop        # [-]               | Vertical dillution of precision.
float64           rms_range   # [-]               | RMS value of the standard deviation of the range inputs to the navigation process.
float64           std_major   # [m]               | Standard deviation of semi-major axis of error ellipse.
float64           std_minor   # [m]               | Standard deviation of semi-minor axis of error ellipse.
float64           angle_major # [deg]             | Angle of semi-major axis of error ellipse from true North.
float64           std_lat     # [m]               | Standard deviation of latitude error.
float64           std_lon     # [m]               | Standard deviation of longitude error.
float64           std_alt     # [m]               | Standard deviation of altitude error.
float64[9]        covariance  # [m2]              | Position covariance defined relative to a tangential plane (ENU frame).
int8              cov_type    # [-]               | Method employed to estimate covariance (see below).
int16             num_sats    # [-]               | Total number of satellites in view.
int16[]           sat_id      # [-]               | Satellite ID number.               
int16[]           elev        # [deg]             | Satellite elevation. 
int16[]           azim        # [deg]             | Satellite azimuth from true North.
int16[]           cno         # [db-Hz]           | Satellite SNR (C/No).
float64           heading     # [deg]             | True heading.
float64           speed       # [m/s]             | Speed over ground.
float64           course      # [deg]             | Course over ground (w.r.t. True North).
float64           diff_age    # [-]               | Approximate age of differential data (last GPS MSM message received).
string            diff_sta    # [-]               | DGPS station ID (0000-1023).
string            sentence    # [-]               | ASCII string to be used by NTRIP clients.

# Quality indicator table
#
# | ID | Signal         |
# |----|----------------|
# |  0 | Invalid        |
# |  1 | Non-RTK fix    |
# |  4 | RTK fixed      |
# |  5 | RTK float      |
# |  6 | Dead-reckoning |


# Covariance type table
#
# | ID | Signal         |
# |----|----------------|
# |  0 | Unknown        |
# |  1 | Approximated   |
# |  2 | Diagonal known |
# |  3 | Known          |
