# Copyright (c) Fixposition AG (www.fixposition.com) and contributors
# License: see the LICENSE file
#
# NOV_B-HEADING2 data

std_msgs/Header header
uint32 sol_status                  # See NovbGnssSolStat
uint32 pos_type                    # See NovbPosOrVelType
float32 length                     # Baseline length (m)
float32 heading                    # Heading (degrees)
float32 pitch                      # Pitch (degrees)
float32 reserved                   # Reserved
float32 heading_stdev              # Heading standard deviation (degrees)
float32 pitch_stdev                # Pitch standard deviation (degrees)
uint8[4] rover_stn_id              # Rover station ID
uint8[4] master_stn_id             # Master station ID
uint8 num_sv_tracked               # Number of satellites tracked
uint8 num_sv_in_sol                # Number of satellites in solution
uint8 num_sv_obs                   # Number of satellites with observations
uint8 num_sv_multi                 # Number of satellites with multi-frequency observations
uint8 sol_source                   # Solution source (see NovbSolSource)
uint8 ext_sol_status               # Extended solution status (see NovbGnssSolExtStat)
uint8 galileo_beidou_sig_mask      # Galileo and BeiDou signal mask (see NovbSigUsedGalBds)
uint8 gps_glonass_sig_mask         # GPS and GLONASS signal mask (see NovbSigUsedGpsGlo)
