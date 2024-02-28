####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition WheelSensor Message
#
#
####################################################################################################
# Standard metadata
std_msgs/Header header

# Location of the wheelspeed measurement (one of: RC, FR, FL, RR, RL)
string location

# Velocity values in [mm/s] as integer 32bit
int32 vx
int32 vy
int32 vz

# Velocity validity
bool vx_valid
bool vy_valid
bool vz_valid
