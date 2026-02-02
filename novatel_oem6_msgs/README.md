# novatel_oem6_msgs


### Created Messages

All messages defined in `gnss_messages_novatel.h` with `GNSS_BINARY_MSG_DEFINITION_START` have been converted to ROS2 messages:

| Message | Purpose | Key Fields |
|---------|---------|------------|
| **BESTPOS** | Best available position | lat, lon, hgt, solution_stat, position_type |
| **INSPVAS** | INS solution | position, velocity (NEU), attitude (RPY), ins_status |
| **INSCOVS** | INS covariances | pos_cov[9], att_cov[9], vel_cov[9] |
| **RAWIMUS** | Raw IMU data | accel_x/y/z, gyro_x/y/z (raw counts) |
| **MARKPOS** | Position at mark event | Same fields as BESTPOS |
| **MARKTIME** | Time at mark 1 event | week, week_seconds, clock_offset |
| **MARK2TIME** | Time at mark 2 event | week, week_seconds, clock_offset |
| **IONUTC** | Ionosphere/UTC params | alpha/beta coefficients, leap seconds |
| **RAWEPHEM** | Raw GPS ephemeris | subframe1[30], subframe2[30], subframe3[30] |
| **RXSTATUS** | Receiver status | error codes, status words |

### Message Features

#### Constants for Status Fields
All messages include constants for their status/type enumerations:

```python
# INSPVAS INS Status
INS_INACTIVE = 0
INS_ALIGNING = 1
INS_SOLUTION_GOOD = 3
# ... etc

# BESTPOS Solution Status
SOLUTION_COMPUTED = 0
INSUFFICIENT_OBS = 1
# ... etc

# BESTPOS Position Type  
NONE = 0
NARROW_INT = 50  # RTK Fixed
L1_FLOAT = 32    # RTK Float
# ... etc
```

#### Standard ROS2 Header
All messages include `std_msgs/Header header` with:
- `frame_id`: Sensor frame
- `stamp`: GPS timestamp converted to ROS time
