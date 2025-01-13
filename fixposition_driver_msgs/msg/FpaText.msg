# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# FP_A-TEXT data

std_msgs/Header header
int8   level   # Level, see consts.TEXT_LEVEL_...
string text    # Text

fixposition_driver_msgs/FpaConsts consts # Constants for enums used in FP_A
