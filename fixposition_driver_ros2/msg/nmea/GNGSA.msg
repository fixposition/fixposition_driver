####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition NMEA-GN-GSA Message
#
#
####################################################################################################

# Format | Field    | Unit | Description
# -------|----------|------|----------------------------------------------------------------------------------------------|
char       mode_op  #  [-] | Operation mode: always A (automatic, allowed to automatically switch 2D/3D).
int8       mode_nav #  [-] | Navigation mode: 1 (fix not available), 2 (2D) or 3 (3D).
int8[]     ids      #  [-] | ID numbers of satellites used in solution. See the NMEA 0183 version 4.11 standard document.
float64    pdop     #  [-] | Position dillution of precision.
float64    hdop     #  [-] | Horizontal dillution of precision.
float64    vdop     #  [-] | Vertical dillution of precision.
int8       gnss_id  #  [-] | GNSS system ID: 1 (GPS, SBAS), 2 (GLONASS), 3 (Galileo), 4 (BeiDou), 5 (QZSS).
