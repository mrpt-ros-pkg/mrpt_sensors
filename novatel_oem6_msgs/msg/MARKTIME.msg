# Novatel OEM6 MARKTIME message
# Time at which a user event mark occurred

std_msgs/Header header

# GPS week number
uint32 week

# Seconds into the GPS week
float64 week_seconds

# Clock offset from GPS time [s]
float64 clock_offset

# Clock offset standard deviation [s]
float64 clock_offset_std

# UTC offset from GPS time [s]
float64 utc_offset

# Clock status word
uint32 clock_status
