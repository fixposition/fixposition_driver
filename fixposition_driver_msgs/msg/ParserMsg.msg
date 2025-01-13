# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# Similar to a fpsdk::common::parser::ParserMsg
#
# !!! fpsdk_{ros1,ros2}/msg/ParserMsg.msg should be kept in sync !!!

std_msgs/Header header            # Timestamp of message reception time
int8    protocol                  # Protocol
int8    PROTOCOL_UNSPECIFIED = 0
int8    PROTOCOL_FP_A        = 1
int8    PROTOCOL_FP_B        = 2
int8    PROTOCOL_NMEA        = 3
int8    PROTOCOL_UBX         = 4
int8    PROTOCOL_RTCM3       = 5
int8    PROTOCOL_UNI_B       = 6
int8    PROTOCOL_NOV_B       = 7
int8    PROTOCOL_SPARTN      = 8
int8    PROTOCOL_OTHER       = 9
uint8[] data                      # Message data
string  name                      # Message name
uint64  seq                       # Message counter
string  info                      # Message (debug) info
