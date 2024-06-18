####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition NMEA-GP-VTG Message
#
#
####################################################################################################

# Format | Field      | Unit    | Description
# -------|------------|---------|--------------------------------------------------------|
float64    cog_true   # [deg]   | Course over ground wrt. True North.
char       cog_ref_t  # [-]     | COG reference, always T (true).
float64    cog_mag    # [-]     | Course over ground w.r.t. Magnetic North, always null.
char       cog_ref_m  # [-]     | COG reference, always M (magnetic).
float64    sog_knot   # [knots] | Speed over ground in knots.
char       sog_unit_n # [-]     | SOG reference, always N (knots).
float64    sog_kph    # [km/h]  | Speed over ground in km/h.
char       sog_unit_k # [-]     | SOG reference, always K (km/h).
char       mode       # [-]     | Positioning system mode indicator (see below).

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
