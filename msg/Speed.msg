####################################################################################################
#
#    Copyright (c) 2021  ___     ___
#                       \\  \\  /  /
#                        \\  \\/  /
#                         /  /\\  \\
#                        /__/  \\__\\  Fixposition AG
#
####################################################################################################
#
# Wheel speed input to Fixposition ROS Driver
#
#
####################################################################################################

# Velocity values in [mm/s] as integer 32bit
# 2 Options:
# Option 1: only one vehicle speed, then only fill a single value as the vehicle speed
# Option 2: Fill in 4 Values of 4 wheels, in the order of FR, FL, RR, RL
int32[] speeds

