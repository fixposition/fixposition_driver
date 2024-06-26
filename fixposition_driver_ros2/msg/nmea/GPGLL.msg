####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition NMEA-GP-GLL Message
#
#
####################################################################################################

# Format | Field     | Unit              | Description
# -------|-----------|-------------------|--------------------------------------------------------|
float64    latitude  # [ddmm.mmmmm(mm)]  | Latitude.
char       lat_ns    # [-]               | Latitude north (N) or south (S) indicator.
float64    longitude # [dddmm.mmmmm(mm)] | Longitude.
char       lon_ew    # [-]               | Longitude east (E) or west (W) indicator.
string     time      # [hhmmss.ss(ss)]   | UTC time (hours, minutes and seconds).
char       status    # [-]               | Data validity status, A (data valid) or V (not valid).
char       mode      # [-]               | Positioning system mode indicator (see below).

# Mode table
#
# | ID | Signal                |
# |----|-----------------------|
# |  N | Data not valid        |
# |  E | Dead-reckoning        |
# |  D | Differential          |
# |  A | Autonomous            |
# |  M | Manual input not used |
# |  S | Simulator not used    |
