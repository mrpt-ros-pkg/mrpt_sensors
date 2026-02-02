# Novatel OEM6 IONUTC message
# Ionospheric and UTC correction parameters

std_msgs/Header header

# Ionospheric alpha parameters
float64 a0
float64 a1
float64 a2
float64 a3

# Ionospheric beta parameters
float64 b0
float64 b1
float64 b2
float64 b3

# UTC reference week number
uint32 utc_wn

# Reference time of UTC params [s]
uint32 tot

# UTC constant term [s]
float64 a0_utc

# UTC 1st order term [s/s]
float64 a1_utc

# Future week number for leap second
uint32 wn_lsf

# Day number (1=Sunday, 7=Saturday)
uint32 dn

# Delta time due to leap seconds [s]
uint32 deltat_ls

# Delta time due to leap seconds (future) [s]
uint32 deltat_lsf

# Reserved
uint32 reserved
