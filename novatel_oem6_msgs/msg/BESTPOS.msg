# Novatel OEM6 BESTPOS message
# Best available position and accuracy information

std_msgs/Header header

# Solution status
uint32 SOLUTION_COMPUTED=0
uint32 INSUFFICIENT_OBS=1
uint32 NO_CONVERGENCE=2
uint32 SINGULARITY=3
uint32 COV_TRACE=4
uint32 TEST_DIST=5
uint32 COLD_START=6
uint32 V_H_LIMIT=7
uint32 VARIANCE=8
uint32 RESIDUALS=9
uint32 DELTA_POS=10
uint32 NEGATIVE_VAR=11
uint32 INTEGRITY_WARNING=13
uint32 INS_INACTIVE=14
uint32 INS_ALIGNING=15
uint32 INS_BAD=16
uint32 IMU_UNPLUGGED=17
uint32 PENDING=18
uint32 INVALID_FIX=19

uint32 solution_stat

# Position type
uint32 NONE=0
uint32 FIXEDPOS=1
uint32 FIXEDHEIGHT=2
uint32 FLOATCONV=4
uint32 WIDELANE=5
uint32 NARROWLANE=6
uint32 DOPPLER_VELOCITY=8
uint32 SINGLE=16
uint32 PSRDIFF=17
uint32 WAAS=18
uint32 PROPAGATED=19
uint32 OMNISTAR=20
uint32 L1_FLOAT=32
uint32 IONOFREE_FLOAT=33
uint32 NARROW_FLOAT=34
uint32 L1_INT=48
uint32 WIDE_INT=49
uint32 NARROW_INT=50
uint32 RTK_DIRECT_INS=51
uint32 INS=52
uint32 INS_PSRSP=53
uint32 INS_PSRDIFF=54
uint32 INS_RTKFLOAT=55
uint32 INS_RTKFIXED=56
uint32 OMNISTAR_HP=64
uint32 OMNISTAR_XP=65
uint32 CDGPS=66

uint32 position_type

# Position [deg, deg, m above sea level]
float64 lat
float64 lon
float64 hgt

# Undulation [m]
float32 undulation

# Datum ID
uint32 datum_id

# Position uncertainties [m]
float32 lat_sigma
float32 lon_sigma
float32 hgt_sigma

# Base station ID
string base_station_id  # 4 chars

# Differential age [s]
float32 diff_age

# Solution age [s]
float32 sol_age

# Number of satellites
uint8 num_sats_tracked
uint8 num_sats_sol
uint8 num_sats_sol_l1
uint8 num_sats_sol_multi

# Reserved
uint8 reserved

# Extended solution status
uint8 ext_sol_stat

# Signal masks
uint8 galileo_beidou_mask
uint8 gps_glonass_mask
