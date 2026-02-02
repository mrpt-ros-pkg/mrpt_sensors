# Novatel OEM6 RAWEPHEM message
# Raw GPS ephemeris data (subframes 1-3)

std_msgs/Header header

# Satellite PRN number
uint32 sat_prn

# Reference GPS week
uint32 ref_week

# Reference GPS seconds
uint32 ref_secs

# GPS navigation message subframes (each 30 bytes)
# Subframe 1: Clock correction, health, and accuracy
uint8[30] subframe1

# Subframe 2: Ephemeris parameters (1st half)
uint8[30] subframe2

# Subframe 3: Ephemeris parameters (2nd half)
uint8[30] subframe3
