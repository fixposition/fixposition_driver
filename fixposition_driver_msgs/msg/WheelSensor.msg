# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# Individual wheelspeed measurement

# Standard metadata
std_msgs/Header header

# Location of the wheelspeed measurement (one of: RC, FR, FL, RR, RL)
string location

# Velocity values in [mm/s]
int32 vx
int32 vy
int32 vz

# Velocity validity
bool vx_valid
bool vy_valid
bool vz_valid
