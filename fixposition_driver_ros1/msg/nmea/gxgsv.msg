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
# Fixposition NMEA-GX-GSV Message
#
#
####################################################################################################

# Format | Field     | Unit    | Description
# -------|-----------|---------|-------------------------------------|
int16      sentences # [-]     | Total number of sentences (1 to 9).
int16      sent_num  # [-]     | Sentence number (1 to 9).
int16      num_sats  # [-]     | Total number of satellites in view.
int16[]    sat_id    # [-]     | Satellite ID number.               
int16[]    elev      # [deg]   | Satellite elevation. 
int16[]    azim      # [deg]   | Satellite azimuth from true North.
int16[]    cno       # [db-Hz] | Satellite SNR (C/No).
string     signal_id # [Hex]   | Signal ID (see below).

# Signal ID table
#
# | ID | Signal                        |
# |----|-------------------------------|
# |  0 | No signal (any talker ID)     |
# |  1 | GPS/SBAS L1C/A (talker ID GP) |
# |  6 | GPS L2C-L (talker ID GP)      |
# |  7 | Galileo L1-BC (talker ID GA)  |
# |  2 | Galileo E5b (talker ID GA)    |
# |  1 | BeiDou B1I (talker ID GB)     |
# |  B | BeiDou B2I (talker ID GB)     |
# |  1 | GLONASS G1 C/A (talker ID GL) |
# |  3 | GLONASS G2 C/A (talker ID GL) |