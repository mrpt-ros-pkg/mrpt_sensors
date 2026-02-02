# Novatel OEM6 INSPVAS message
# Integrated navigation system position, velocity, and attitude solution

std_msgs/Header header

# GPS week number
uint32 week

# Seconds into the GPS week
float64 seconds_in_week

# Position [deg, deg, m above sea level]
float64 lat
float64 lon
float64 hgt

# Velocity North-East-Up [m/s]
float64 vel_north
float64 vel_east
float64 vel_up

# Attitude [deg]
float64 roll
float64 pitch
float64 azimuth

# INS status constants
uint32 INS_INACTIVE=0
uint32 INS_ALIGNING=1
uint32 INS_HIGH_VARIANCE=2
uint32 INS_SOLUTION_GOOD=3
uint32 INS_SOLUTION_FREE=6
uint32 INS_ALIGNMENT_COMPLETE=7
uint32 INS_DETERMINING_ORIENTATION=8
uint32 INS_WAITING_INITIALPOS=9
uint32 INS_WAITING_AZIMUTH=10
uint32 INS_INITIALIZING_BIASES=11
uint32 INS_MOTION_DETECT=12

uint32 ins_status
