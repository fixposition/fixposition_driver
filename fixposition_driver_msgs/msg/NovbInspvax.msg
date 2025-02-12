# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NOV_B-INSPVAX data

std_msgs/Header header
uint32 ins_status               # See NovbInsSolStatus
uint32 pos_type                 # See NovbPosOrVelType
float32 latitude                # Latitude (degrees)
float32 longitude               # Longitude (degrees)
float32 height                  # Height above mean sea level (m)
float32 undulation              # Undulation (m)
float32 north_velocity          # North velocity (m/s)
float32 east_velocity           # East velocity (m/s)
float32 up_velocity             # Up velocity (m/s)
float32 roll                    # Roll in Local Level (degrees)
float32 pitch                   # Pitch in Local Level (degrees)
float32 azimuth                 # Azimuth in Local Level (degrees)
float32 latitude_stdev          # Latitude standard deviation (m)
float32 longitude_stdev         # Longitude standard deviation (m)
float32 height_stdev            # Height standard deviation (m)
float32 north_velocity_stdev    # North velocity standard deviation (m/s)
float32 east_velocity_stdev     # East velocity standard deviation (m/s)
float32 up_velocity_stdev       # Up velocity standard deviation (m/s)
float32 roll_stdev              # Roll standard deviation (degrees)
float32 pitch_stdev             # Pitch standard deviation (degrees)
float32 azimuth_stdev           # Azimuth standard deviation (degrees)
uint32 extended_status          # Extended solution status
uint16 time_since_update        # Elapsed time since the last position update (seconds)
