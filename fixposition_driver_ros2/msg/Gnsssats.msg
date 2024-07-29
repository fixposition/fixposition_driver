####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition GNSS satellite signal statistics message.
#
#
####################################################################################################

# Format | Field         | Unit    | Description
# -------|---------------|---------|------------------------------------|
string     constellation # [Hex]   | Signal ID (see below).
int16[]    sat_id        # [-]     | Satellite ID number.
int16[]    azim          # [deg]   | Satellite azimuth from true North.
int16[]    elev          # [deg]   | Satellite elevation.
int16[]    cno_l1        # [db-Hz] | Satellite SNR (C/No) for L1-band.
int16[]    cno_l2        # [db-Hz] | Satellite SNR (C/No) for L2-band.
