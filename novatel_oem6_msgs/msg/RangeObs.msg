# One tracked signal, decoded from a Novatel OEM6 RANGECMP record

# Satellite system
uint8 SYSTEM_GPS=0
uint8 SYSTEM_GLONASS=1
uint8 SYSTEM_SBAS=2
uint8 SYSTEM_GALILEO=3
uint8 SYSTEM_BEIDOU=4
uint8 SYSTEM_QZSS=5

uint8 system

# Signal type, depends on the system.
# GPS: 0=L1 C/A, 5=L2 P, 9=L2 P codeless, 17=L2C. GLONASS: 0=L1 C/A, 5=L2 P.
# SBAS: 0=L1 C/A.
uint8 signal_type

# Satellite PRN (GLONASS: slot + 37, or 0 while the slot is still unknown)
uint16 prn

# GLONASS frequency channel number with +7 offset (0 for other systems)
uint8 glonass_freq

# Carrier to noise density ratio [dB-Hz] (1 dB resolution, range 20 to 51)
float32 cno

# Time of continuous tracking without loss of lock [s]
float32 lock_time

# Pseudorange and its standard deviation [m]
float64 pseudorange
float32 pseudorange_std

# Doppler [Hz]
float64 doppler

# Flags from the channel tracking status
bool phase_locked
bool code_locked

# Raw channel tracking status word
uint32 tracking_status
