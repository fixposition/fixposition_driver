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
# Fixposition FP_A-GNSSANT Message
#
#
####################################################################################################

Header header
string gnss1_state   # GNSS1 antenna state
string gnss1_power   # GNSS1 antenna power
int16  gnss1_age     # Time since gnss1_state or gnss1_power information last changed
string gnss2_state   # GNSS2 antenna state
string gnss2_power   # GNSS2 antenna power
int16  gnss2_age     # Time since gnss2_state or gnss2_power information last changed
