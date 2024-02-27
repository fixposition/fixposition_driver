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
# Individual wheelspeed measurement
#
#
####################################################################################################
# Location of the wheelspeed measurement (one of: RC, FR, FL, RR, RL)
string location

# Velocity values in [mm/s] as integer 32bit
# Options:
# Option 1: One value, will be input in the x axis
# Option 2: Two values, will be input in the x and y axis
# Option 2: Three values, will be input in the x, y, and z axis
int32[] speeds
