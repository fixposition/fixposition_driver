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
# Fixposition NMEA-GP-ZDA Message
#
#
####################################################################################################

# Format | Field     | Unit             | Description
# -------|-----------|------------------|----------------------------------------|
string     time      #  [hhmmss.ss(ss)] | UTC time (hours, minutes and seconds).
int16      day       #  [01-31]         | Day (UTC).
int16      month     #  [01-12]         | Month (UTC).
int16      year      #  [yyyy]          | Year (UTC).
int16      local_hr  #  [00+-13]        | Local zone hours, always 00 (= UTC).
int16      local_min #  [00-59]         | Local zone minutes, always 00 (= UTC).
