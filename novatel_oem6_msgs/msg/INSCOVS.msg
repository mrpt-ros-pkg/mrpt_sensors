# Novatel OEM6 INSCOVS message
# INS covariance matrix for position, attitude, and velocity

std_msgs/Header header

# GPS week number
uint32 week

# Seconds into the GPS week
float64 seconds_in_week

# Position covariance matrix in local level frame [m^2]
# 3x3 matrix stored row-major: xx, xy, xz, yx, yy, yz, zx, zy, zz
float64[9] pos_cov

# Attitude covariance matrix of SPAN frame to local level frame [deg^2]
# 3x3 matrix stored row-major: xx, xy, xz, yx, yy, yz, zx, zy, zz
float64[9] att_cov

# Velocity covariance matrix in local level frame [m^2/s^2]
# 3x3 matrix stored row-major: xx, xy, xz, yx, yy, yz, zx, zy, zz
float64[9] vel_cov
