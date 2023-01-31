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
# Wheel speed input to Fixposition ROS Driver
#
#
####################################################################################################

# Velocity values in [mm/s] (or [mrad/s]) as integer 32bit
# 2 Options:
# Option 1: One vehicle speed, of sensor RC
# Option 2: One vehicle speed, of sensor RC and the yaw rate of the vehicle
# Option 3: Fill in 4 Values of 4 wheels, in the order of FR, FL, RR, RL
int32[] speeds
