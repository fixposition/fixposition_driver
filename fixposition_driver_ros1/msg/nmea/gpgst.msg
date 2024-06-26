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
# Fixposition NMEA-GP-GST Message
#
#
####################################################################################################

# Format | Field       | Unit            | Description
# -------|-------------|-----------------|------------------------------------------------------------------------------------|
string     time        # [hhmmss.ss(ss)] | UTC time (hours, minutes and seconds).
float64    rms_range   # [-]             | RMS value of the standard deviation of the range inputs to the navigation process.
float64    std_major   # [m]             | Standard deviation of semi-major axis of error ellipse.
float64    std_minor   # [m]             | Standard deviation of semi-minor axis of error ellipse.
float64    angle_major # [deg]           | Angle of semi-major axis of error ellipse from true North.
float64    std_lat     # [m]             | Standard deviation of latitude error.
float64    std_lon     # [m]             | Standard deviation of longitude error.
float64    std_alt     # [m]             | Standard deviation of altitude error.
