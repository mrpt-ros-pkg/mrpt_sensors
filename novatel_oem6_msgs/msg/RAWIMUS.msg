# Novatel OEM6 RAWIMUS message
# Raw IMU measurements (short header format)

std_msgs/Header header

# GPS week number
uint32 week

# Seconds into the GPS week
float64 week_seconds

# IMU status word
uint32 imu_status

# Raw IMU measurements (counts/ticks from IMU)
# These are raw sensor outputs that need scaling factors from IMU specs
int32 accel_z
int32 accel_y_neg  # Negative Y acceleration
int32 accel_x

int32 gyro_z
int32 gyro_y_neg   # Negative Y gyro rate
int32 gyro_x
