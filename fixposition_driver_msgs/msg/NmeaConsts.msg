# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NMEA enum constants

# NmeaTalkerId
int8   TALKER_ID_UNSPECIFIED                       =  -1  # Unspecified
int8   TALKER_ID_PROPRIETARY                       =   1  # Proprietary
int8   TALKER_ID_GPS_SBAS                          =   2  # GPS and/or SBAS
int8   TALKER_ID_GLO                               =   3  # GLONASS
int8   TALKER_ID_GAL                               =   4  # GALILEO
int8   TALKER_ID_BDS                               =   5  # BeiDou
int8   TALKER_ID_NAVIC                             =   6  # NavIC
int8   TALKER_ID_QZSS                              =   7  # QZSS
int8   TALKER_ID_GNSS                              =   8  # GNSS (multi-constellation)

# NmeaQualityGga
int8   QUALITY_GGA_UNSPECIFIED                     =  -1  # Unspecified
int8   QUALITY_GGA_NOFIX                           =   0  # No fix
int8   QUALITY_GGA_SPP                             =   1  # Autonomous GNSS fix
int8   QUALITY_GGA_DGNSS                           =   2  # Differential GPS fix (e.g. with SBAS)
int8   QUALITY_GGA_PPS                             =   3  # PPS mode
int8   QUALITY_GGA_RTK_FIXED                       =   4  # RTK fixed
int8   QUALITY_GGA_RTK_FLOAT                       =   5  # RTK float
int8   QUALITY_GGA_ESTIMATED                       =   6  # Estimated (dead reckoning only)
int8   QUALITY_GGA_MANUAL                          =   7  # Manual input mode
int8   QUALITY_GGA_SIM                             =   8  # Simulator

# NmeaStatusGllRmc
int8   STATUS_GLL_RMC_UNSPECIFIED                  =  -1  # Unspecified
int8   STATUS_GLL_RMC_INVALID                      =   0  # Data invalid
int8   STATUS_GLL_RMC_VALID                        =   1  # Data valid

# NmeaModeGllVtg
int8   MODE_GLL_VTG_UNSPECIFIED                    =  -1  # Unspecified
int8   MODE_GLL_VTG_INVALID                        =   0  # Invalid (no fix)
int8   MODE_GLL_VTG_AUTONOMOUS                     =   1  # Autonomous mode (SPP)
int8   MODE_GLL_VTG_DGNSS                          =   2  # Differential GNSS fix
int8   MODE_GLL_VTG_ESTIMATED                      =   3  # Estimated (dead reckoning only)
int8   MODE_GLL_VTG_MANUAL                         =   4  # Manual input mode
int8   MODE_GLL_VTG_SIM                            =   5  # Simulator mode

# NmeaModeRmcGns
int8   MODE_RMC_GNS_UNSPECIFIED                    =  -1  # Unspecified
int8   MODE_RMC_GNS_INVALID                        =   0  # Invalid (no fix)
int8   MODE_RMC_GNS_AUTONOMOUS                     =   1  # Autonomous mode (SPP)
int8   MODE_RMC_GNS_DGNSS                          =   2  # Differential GNSS fix
int8   MODE_RMC_GNS_ESTIMATED                      =   3  # Estimated (dead reckoning only)
int8   MODE_RMC_GNS_RTK_FIXED                      =   4  # RTK fixed
int8   MODE_RMC_GNS_RTK_FLOAT                      =   5  # RTK float
int8   MODE_RMC_GNS_PRECISE                        =   6  # Precise
int8   MODE_RMC_GNS_MANUAL                         =   7  # Manual input mode
int8   MODE_RMC_GNS_SIM                            =   8  # Simulator mode

# NmeaNavStatusRmc
int8   NAV_STATUS_RMC_UNSPECIFIED                  =  -1  # Unspecified
int8   NAV_STATUS_RMC_NA                           =   0  # Equipment does not provide navigational status
int8   NAV_STATUS_RMC_SAFE                         =   1  # Safe
int8   NAV_STATUS_RMC_CAUTION                      =   2  # Caution
int8   NAV_STATUS_RMC_UNSAFE                       =   3  # Unsafe

# NmeaOpModeGsa
int8   OP_MODE_GSA_UNSPECIFIED                     =  -1  # Unspecified
int8   OP_MODE_GSA_MANUAL                          =   1  # Manual
int8   OP_MODE_GSA_AUTO                            =   2  # Automatic

# NmeaNavModeGsa
int8   NAV_MODE_GSA_UNSPECIFIED                    =  -1  # Unspecified
int8   NAV_MODE_GSA_NOFIX                          =   1  # No fix
int8   NAV_MODE_GSA_FIX2D                          =   2  # 2D fix
int8   NAV_MODE_GSA_FIX3D                          =   3  # 3D fix

# NmeaSystemId
int8   SYSTEM_ID_UNSPECIFIED                       =  -1  # Unspecified
int8   SYSTEM_ID_GPS_SBAS                          =   1  # GPS and/or SBAS
int8   SYSTEM_ID_GLO                               =   2  # GLONASS
int8   SYSTEM_ID_GAL                               =   3  # Galileo
int8   SYSTEM_ID_BDS                               =   4  # BeiDou
int8   SYSTEM_ID_QZSS                              =   5  # QZSS
int8   SYSTEM_ID_NAVIC                             =   6  # NavIC

# NmeaSignalId
int8   SIGNAL_ID_UNSPECIFIED                       =  -1  # Unspecified
int8   SIGNAL_ID_NONE                              =   0  # None
int8   SIGNAL_ID_GPS_L1CA                          =  11  # GPS/SBAS L1 C/A or SBAS L1 C/A
int8   SIGNAL_ID_GPS_L2CL                          =  12  # GPS L2 CL
int8   SIGNAL_ID_GPS_L2CM                          =  13  # GPS L2 CM
int8   SIGNAL_ID_GPS_L5I                           =  14  # GPS L5 I
int8   SIGNAL_ID_GPS_L5Q                           =  15  # GPS L5 Q
int8   SIGNAL_ID_GAL_E1                            =  31  # Galileo E1
int8   SIGNAL_ID_GAL_E5A                           =  32  # Galileo E5 A
int8   SIGNAL_ID_GAL_E5B                           =  33  # Galileo E5 B
int8   SIGNAL_ID_BDS_B1ID                          =  41  # BeiDou B1I D
int8   SIGNAL_ID_BDS_B2ID                          =  42  # BeiDou B2I D
int8   SIGNAL_ID_BDS_B1C                           =  43  # BeiDou B1 C
int8   SIGNAL_ID_BDS_B2A                           =  44  # BeiDou B2 a
int8   SIGNAL_ID_QZSS_L1CA                         =  51  # QZSS L1 C/A
int8   SIGNAL_ID_QZSS_L1S                          =  52  # QZSS L1S
int8   SIGNAL_ID_QZSS_L2CM                         =  53  # QZSS L2 CM
int8   SIGNAL_ID_QZSS_L2CL                         =  54  # QZSS L2 CL
int8   SIGNAL_ID_QZSS_L5I                          =  55  # QZSS L5 I
int8   SIGNAL_ID_QZSS_L5Q                          =  56  # QZSS L5 Q
int8   SIGNAL_ID_GLO_L1OF                          =  21  # GLONASS L1 OF
int8   SIGNAL_ID_GLO_L2OF                          =  22  # GLONASS L2 OF
int8   SIGNAL_ID_NAVIC_L5A                         =  61  # NavIC L5 A
