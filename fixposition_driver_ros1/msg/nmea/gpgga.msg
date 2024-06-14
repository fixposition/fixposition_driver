####################################################################################################
#
#    Copyright (c) 2023  ___     ___
#                       \\  \\  /  /
#                        \\  \\/  /
#                         /  /\\  \\
#                        /__/  \\__\\  Fixposition AG
#
####################################################################################################
#
# Fixposition NMEA-GP-GGA Message
#
#
####################################################################################################

# Format | Field     | Unit              | Description
# -------|-----------|-------------------|----------------------------------------------------------------------|
string     time      # [hhmmss.ss(ss)]   | UTC time (hours, minutes and seconds).
float64    latitude  # [ddmm.mmmmm(mm)]  | Latitude.
char       lat_ns    # [-]               | Latitude north (N) or south (S) indicator.
float64    longitude # [dddmm.mmmmm(mm)] | Longitude.
char       lon_ew    # [-]               | Longitude east (E) or west (W) indicator.
int8       quality   # [-]               | Quality indicator (see below).
int8       num_sv    # [-]               | Number of satellites. Strict NMEA: 00-12. High-precision NMEA: 00-99.
float64    hdop      # [0.10-99.99]      | Horizontal dilution of precision.
float64    alt       # [m]               | Altitude (above ellipsoid).
char       alt_unit  # [-]               | Altitude unit, always M (metres).
float64    diff_age  # [-]               | Approximate age of differential data (last GPS MSM message received).
string     diff_sta  # [-]               | DGPS station ID (0000-1023).

# Quality indicator table
#
# | ID | Signal         |
# |----|----------------|
# |  0 | Invalid        |
# |  1 | Non-RTK fix    |
# |  4 | RTK fixed      |
# |  5 | RTK float      |
# |  6 | Dead-reckoning |
