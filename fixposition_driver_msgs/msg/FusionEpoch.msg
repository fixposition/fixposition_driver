# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# Fusion epoch data

##### Fusion epoch data

std_msgs/Header header                                        # Header (only the timestamp is of interest)
bool                                   fpa_odometry_avail     # FP_A-ODOMETRY data available
fixposition_driver_msgs/FpaOdometry    fpa_odometry           # FP_A-ODOMETRY data (only valid if fpa_odometry_avail = true)
bool                                   fpa_odomsh_avail       # FP_A-ODOMSH data available
fixposition_driver_msgs/FpaOdomsh      fpa_odomsh             # FP_A-ODOMSH data (only valid if fpa_odomsh_avail = true)
bool                                   fpa_odomenu_avail      # FP_A-ODOMENU data available
fixposition_driver_msgs/FpaOdomenu     fpa_odomenu            # FP_A-ODOMENU data (only valid if fpa_odomenu_avail = true)
bool                                   fpa_odomstatus_avail   # FP_A-ODOMSTATUS data available
fixposition_driver_msgs/FpaOdomstatus  fpa_odomstatus         # FP_A-ODOMSTATUS data (only valid if fpa_odomstatus_avail = true)
bool                                   novb_inspvax_avail     # NOV_B-INSPVAX data available
fixposition_driver_msgs/NovbInspvax    novb_inspvax           # NOV_B-INSPVAX data (only valid if novb_inspvax_avail = true)


##### Other data. These may have a lower update rate than the epoch data above. Check the timestamps!

bool                                   fpa_imubias_avail      # FP_A-ODOMETRY data available
fixposition_driver_msgs/FpaImubias     fpa_imubias            # FP_A-ODOMETRY data (only valid if fpa_imubias_avail = true)
