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
# Fixposition FP_A-TP Message
#
#
####################################################################################################

string  tp_name                              # Timepulse name (source)
string  timebase                             # Time base (see below), or null if not available
string  timeref                              # Time reference (see below), or null if not available
int64   tp_tow_sec                           # Timepulse time seconds of week, integer second part (0–604799), or null
float64 tp_tow_psec                          # Timepulse time seconds of week, sub-second part (0.000000000000–0.999999999999), or null
int64   gps_leaps                            # GPS leapseconds, or null if unknown


# Values for timebase
#
# | Value | Description               |
# |-------|---------------------------|
# |  null | No timepulse alignment    |
# |  GNSS | Timepulse aligned to GNSS |
# |   UTC | Timepulse aligned to UTC  |


# Values for timeref if timebase is GNSS
#
# | Value | Description                     |
# |-------|---------------------------------|
# |   GPS | Timepulse aligned to GPS        |
# |   GAL | Timepulse aligned to Galileo    |
# |   BDS | Timepulse aligned to BeiDou     |
# |   GLO | Timepulse aligned to GLONASS    |
# | OTHER | Timepulse aligned to other GNSS |


# Values for timeref if timebase is UTC
#
# | Value | Description                                                                |
# |-------|----------------------------------------------------------------------------|
# |  NONE | Timepulse aligned to no UTC (no precise UTC parameters known yet)          |
# |   CRL | Timepulse aligned to Communications Research Laboratory (CRL), Japan       |
# |  NIST | Timepulse aligned to National Institute of Standards and Technology (NIST) |
# |  USNO | Timepulse aligned to U.S. Naval Observatory (USNO)                         |
# |  BIPM | Timepulse aligned to International Bureau of Weights and Measures (BIPM)   |
# |    EU | Timepulse aligned to European laboratories                                 |
# |    SU | Timepulse aligned to Former Soviet Union (SU)                              |
# |  NTSC | Timepulse aligned to National Time Service Center (NTSC), China            |
# | OTHER | Timepulse aligned to other/unknown UTC                                     |
