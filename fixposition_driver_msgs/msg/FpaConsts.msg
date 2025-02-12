# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# FP_A enum constants

# FpaFusionStatusLegacy
int8   FUSION_STATUS_LEGACY_UNSPECIFIED            =  -1  # Unspecified
int8   FUSION_STATUS_LEGACY_NONE                   =   0  # Not started
int8   FUSION_STATUS_LEGACY_VISION                 =   1  # Vision only
int8   FUSION_STATUS_LEGACY_VIO                    =   2  # Visual-inertial fusion
int8   FUSION_STATUS_LEGACY_IMU_GNSS               =   3  # Inertial-GNSS fusion
int8   FUSION_STATUS_LEGACY_VIO_GNSS               =   4  # Visual-inertial-GNSS fusion

# FpaImuStatusLegacy
int8   IMU_STATUS_LEGACY_UNSPECIFIED               =  -1  # Unspecified
int8   IMU_STATUS_LEGACY_NOT_CONVERGED             =   0  # Not converged
int8   IMU_STATUS_LEGACY_CONVERGED                 =   1  # Converged

# FpaWsStatusLegacy
int8   WS_STATUS_LEGACY_UNSPECIFIED                =  -2  # Unspecified
int8   WS_STATUS_LEGACY_NOT_ENABLED                =  -1  # No wheelspeed enabled
int8   WS_STATUS_LEGACY_NONE_CONVERGED             =   0  # None converged
int8   WS_STATUS_LEGACY_ONE_OR_MORE_CONVERGED      =   1  # At least one converged

# FpaInitStatus
int8   INIT_STATUS_UNSPECIFIED                     =  -1  # Unspecified
int8   INIT_STATUS_NOT_INIT                        =   0  # Not initialised
int8   INIT_STATUS_LOCAL_INIT                      =   1  # Locally initialised
int8   INIT_STATUS_GLOBAL_INIT                     =   2  # Globally initialised

# FpaMeasStatus
int8   MEAS_STATUS_UNSPECIFIED                     =  -1  # Unspecified
int8   MEAS_STATUS_NOT_USED                        =   0  # Not used
int8   MEAS_STATUS_USED                            =   1  # Used
int8   MEAS_STATUS_DEGRADED                        =   2  # Degraded

# FpaImuStatus
int8   IMU_STATUS_UNSPECIFIED                      =  -1  # Unspecified
int8   IMU_STATUS_NOT_CONVERGED                    =   0  # Not converged
int8   IMU_STATUS_WARMSTARTED                      =   1  # Warmstarted
int8   IMU_STATUS_ROUGH_CONVERGED                  =   2  # Rough convergence
int8   IMU_STATUS_FINE_CONVERGED                   =   3  # Fine convergence

# FpaImuNoise
int8   IMU_NOISE_UNSPECIFIED                       =  -1  # Unspecified
int8   IMU_NOISE_LOW_NOISE                         =   1  # Low noise
int8   IMU_NOISE_MEDIUM_NOISE                      =   2  # Medium noise
int8   IMU_NOISE_HIGH_NOISE                        =   3  # High noise
int8   IMU_NOISE_RESERVED4                         =   4  # Reserved
int8   IMU_NOISE_RESERVED5                         =   5  # Reserved
int8   IMU_NOISE_RESERVED6                         =   6  # Reserved
int8   IMU_NOISE_RESERVED7                         =   7  # Reserved

# FpaImuConv
int8   IMU_CONV_UNSPECIFIED                        =  -1  # Unspecified
int8   IMU_CONV_RESERVED0                          =   0  # Reserved
int8   IMU_CONV_WAIT_IMU_MEAS                      =   1  # Awaiting IMU measurements
int8   IMU_CONV_WAIT_GLOBAL_MEAS                   =   2  # Insufficient global measurements
int8   IMU_CONV_WAIT_MOTION                        =   3  # Insufficient motion
int8   IMU_CONV_CONVERGING                         =   4  # Converging
int8   IMU_CONV_RESERVED5                          =   5  # Reserved
int8   IMU_CONV_RESERVED6                          =   6  # Reserved
int8   IMU_CONV_IDLE                               =   7  # Idle

# FpaGnssStatus
int8   GNSS_STATUS_UNSPECIFIED                     =  -1  # Unspecified
int8   GNSS_STATUS_NO_FIX                          =   0  # No fix
int8   GNSS_STATUS_SPP                             =   1  # Single-point positioning (SPP)
int8   GNSS_STATUS_RTK_MB                          =   2  # RTK moving baseline
int8   GNSS_STATUS_RESERVED3                       =   3  # Reserved
int8   GNSS_STATUS_RESERVED4                       =   4  # Reserved
int8   GNSS_STATUS_RTK_FLOAT                       =   5  # RTK float
int8   GNSS_STATUS_RESERVED6                       =   6  # Reserved
int8   GNSS_STATUS_RESERVED7                       =   7  # Reserved
int8   GNSS_STATUS_RTK_FIXED                       =   8  # RTK fixed
int8   GNSS_STATUS_RESERVED9                       =   9  # Reserved

# FpaBaselineStatus
int8   BASELINE_STATUS_UNSPECIFIED                 =  -1  # Unspecified
int8   BASELINE_STATUS_WAITING_FUSION              =   0  # Waiting fusion
int8   BASELINE_STATUS_NO_FIX                      =   1  # Not available or no fix
int8   BASELINE_STATUS_FAILING                     =   2  # Failing
int8   BASELINE_STATUS_PASSING                     =   3  # Passing

# FpaCorrStatus
int8   CORR_STATUS_UNSPECIFIED                     =  -1  # Unspecified
int8   CORR_STATUS_WAITING_FUSION                  =   0  # Waiting fusion
int8   CORR_STATUS_NO_GNSS                         =   1  # No GNSS available
int8   CORR_STATUS_NO_CORR                         =   2  # No corrections used
int8   CORR_STATUS_LIMITED_CORR                    =   3  # Limited corrections used: station data & at least one of the constellations (both bands) among the valid rover measurements
int8   CORR_STATUS_OLD_CORR                        =   4  # Corrections are too old (TBD)
int8   CORR_STATUS_GOOD_CORR                       =   5  # Sufficient corrections used: station data and corrections for ALL bands among the valid rover measurements
int8   CORR_STATUS_RESERVED6                       =   6  # Reserved
int8   CORR_STATUS_RESERVED7                       =   7  # Reserved
int8   CORR_STATUS_RESERVED8                       =   8  # Reserved
int8   CORR_STATUS_RESERVED9                       =   9  # Reserved

# FpaCamStatus
int8   CAM_STATUS_UNSPECIFIED                      =  -1  # Unspecified
int8   CAM_STATUS_CAM_UNAVL                        =   0  # Camera not available
int8   CAM_STATUS_BAD_FEAT                         =   1  # Camera available, but not usable  (e.g. too dark)
int8   CAM_STATUS_RESERVED2                        =   2  # Reserved
int8   CAM_STATUS_RESERVED3                        =   3  # Reserved
int8   CAM_STATUS_RESERVED4                        =   4  # Reserved
int8   CAM_STATUS_GOOD                             =   5  # Camera working and available

# FpaWsStatus
int8   WS_STATUS_UNSPECIFIED                       =  -1  # Unspecified
int8   WS_STATUS_NOT_ENABLED                       =   0  # No wheelspeed enabled
int8   WS_STATUS_MISS_MEAS                         =   1  # Missing wheelspeed measurements
int8   WS_STATUS_NONE_CONVERGED                    =   2  # At least one wheelspeed enabled, no wheelspeed converged
int8   WS_STATUS_ONE_CONVERGED                     =   3  # At least one wheelspeed enabled and at least one converged
int8   WS_STATUS_ALL_CONVERGED                     =   4  # At least one wheelspeed enabled and all converged

# FpaWsConv
int8   WS_CONV_UNSPECIFIED                         =  -1  # Unspecified
int8   WS_CONV_WAIT_FUSION                         =   0  # Awaiting Fusion
int8   WS_CONV_WAIT_WS_MEAS                        =   1  # Missing wheelspeed measurements
int8   WS_CONV_WAIT_GLOBAL_MEAS                    =   2  # Insufficient global measurements
int8   WS_CONV_WAIT_MOTION                         =   3  # Insufficient motion
int8   WS_CONV_WAIT_IMU_BIAS                       =   4  # Insufficient imu bias convergence
int8   WS_CONV_CONVERGING                          =   5  # Converging
int8   WS_CONV_IDLE                                =   6  # Idle

# FpaMarkersStatus
int8   MARKERS_STATUS_UNSPECIFIED                  =  -1  # Unspecified
int8   MARKERS_STATUS_NOT_ENABLED                  =   0  # No markers available
int8   MARKERS_STATUS_NONE_CONVERGED               =   1  # Markers available
int8   MARKERS_STATUS_ONE_CONVERGED                =   2  # Markers available and used
int8   MARKERS_STATUS_ALL_CONVERGED                =   3  # All markers available and used

# FpaMarkersConv
int8   MARKERS_CONV_UNSPECIFIED                    =  -1  # Unspecified
int8   MARKERS_CONV_WAIT_FUSION                    =   0  # Awaiting Fusion
int8   MARKERS_CONV_WAIT_MARKER_MEAS               =   1  # Waiting marker measurements
int8   MARKERS_CONV_WAIT_GLOBAL_MEAS               =   2  # Insufficient global measurements
int8   MARKERS_CONV_CONVERGING                     =   3  # Converging
int8   MARKERS_CONV_IDLE                           =   4  # Idle

# FpaEpoch
int8   EPOCH_UNSPECIFIED                           =  -1  # Unspecified
int8   EPOCH_GNSS1                                 =   1  # GNSS 1
int8   EPOCH_GNSS2                                 =   2  # GNSS 2
int8   EPOCH_GNSS                                  =   3  # Combined GNSS
int8   EPOCH_FUSION                                =   4  # Fusion

# FpaAntState
int8   ANT_STATE_UNSPECIFIED                       =  -1  # Unspecified
int8   ANT_STATE_OPEN                              =   0  # No antenna detected (or connected via DC block)
int8   ANT_STATE_OK                                =   1  # Antenna detected and good
int8   ANT_STATE_SHORT                             =   2  # Antenna short circuit detected

# FpaAntPower
int8   ANT_POWER_UNSPECIFIED                       =  -1  # Unspecified
int8   ANT_POWER_OFF                               =   0  # Antenna power supply is off
int8   ANT_POWER_ON                                =   1  # Antenna power supply is on

# FpaGnssFix
int8   GNSS_FIX_UNSPECIFIED                        =  -1  # Unspecified
int8   GNSS_FIX_UNKNOWN                            =   0  # Unknown
int8   GNSS_FIX_NOFIX                              =   1  # No fix
int8   GNSS_FIX_DRONLY                             =   2  # Dead-reckoning only
int8   GNSS_FIX_TIME                               =   3  # Time-only fix
int8   GNSS_FIX_S2D                                =   4  # Single 2D fix
int8   GNSS_FIX_S3D                                =   5  # Single 3D fix
int8   GNSS_FIX_S3D_DR                             =   6  # Single 3D fix with dead-reckoning
int8   GNSS_FIX_RTK_FLOAT                          =   7  # RTK float fix
int8   GNSS_FIX_RTK_FIXED                          =   8  # RTK fixed fix

# FpaTextLevel
int8   TEXT_LEVEL_UNSPECIFIED                      =  -1  # Unspecified
int8   TEXT_LEVEL_ERROR                            =   3  # Error
int8   TEXT_LEVEL_WARNING                          =   4  # Warning
int8   TEXT_LEVEL_INFO                             =   6  # Info
int8   TEXT_LEVEL_DEBUG                            =   7  # Debug

# FpaTimebase
int8   TIMEBASE_UNSPECIFIED                        =  -1  # Unspecified
int8   TIMEBASE_NONE                               =   0  # None
int8   TIMEBASE_GNSS                               =   1  # GNSS
int8   TIMEBASE_UTC                                =   2  # UTC

# FpaTimeref
int8   TIMEREF_UNSPECIFIED                         =  -1  # Unspecified
int8   TIMEREF_UTC_NONE                            =  10  # UTC: None (UTC params not yet known)
int8   TIMEREF_UTC_CRL                             =  11  # UTC: Communications Research Laboratory (CRL), Japan
int8   TIMEREF_UTC_NIST                            =  12  # UTC: National Institute of Standards and Technology (NIST)
int8   TIMEREF_UTC_USNO                            =  13  # UTC: U.S. Naval Observatory (USNO)
int8   TIMEREF_UTC_BIPM                            =  14  # UTC: International Bureau of Weights and Measures (BIPM)
int8   TIMEREF_UTC_EU                              =  15  # UTC: European laboratories
int8   TIMEREF_UTC_SU                              =  16  # UTC: Former Soviet Union (SU)
int8   TIMEREF_UTC_NTSC                            =  17  # UTC: National Time Service Center (NTSC), China
int8   TIMEREF_GNSS_GPS                            =  21  # GNSS: GPS
int8   TIMEREF_GNSS_GAL                            =  22  # GNSS: Galileo
int8   TIMEREF_GNSS_BDS                            =  23  # GNSS: BeiDou
int8   TIMEREF_GNSS_GLO                            =  24  # GNSS: GLONASS
int8   TIMEREF_GNSS_NVC                            =  25  # GNSS: NavIC
int8   TIMEREF_OTHER                               =  99  # other/unknown GNSS/UTC
