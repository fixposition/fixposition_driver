####################################################################################################
#
#    Copyright (c) 2023
#    Fixposition AG
#
####################################################################################################
#
# Fixposition NMEA-GP-ZDA Message
#
#
####################################################################################################

# Format        | Field     | Unit            | Description
# --------------|-----------|-----------------|-------------------------------------------------------|
std_msgs/Header   header    # [ns]            | Specifies the ROS time and Euclidian reference frame.
string            time      # [hhmmss.ss(ss)] | UTC0 time (hours, minutes and seconds).
string            date      # [dd/mm/yyyy]    | Date (UTC0).
int16             local_hr  # [00+-13]        | Local zone hours, always 00 (= UTC).
int16             local_min # [00-59]         | Local zone minutes, always 00 (= UTC).
