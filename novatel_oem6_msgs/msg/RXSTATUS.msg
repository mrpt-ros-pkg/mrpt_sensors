# Novatel OEM6 RXSTATUS message
# Receiver status and error flags

std_msgs/Header header

# Error code
uint32 error

# Number of status codes
uint32 num_stats

# Receiver status words
uint32 rxstat
uint32 rxstat_pri
uint32 rxstat_set
uint32 rxstat_clear

# Auxiliary port 1 status
uint32 aux1stat
uint32 aux1stat_pri
uint32 aux1stat_set
uint32 aux1stat_clear

# Auxiliary port 2 status
uint32 aux2stat
uint32 aux2stat_pri
uint32 aux2stat_set
uint32 aux2stat_clear

# Auxiliary port 3 status
uint32 aux3stat
uint32 aux3stat_pri
uint32 aux3stat_set
uint32 aux3stat_clear
