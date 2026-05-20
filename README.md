[![CI Build colcon](https://github.com/mrpt-ros-pkg/mrpt_sensors/actions/workflows/ros-build.yml/badge.svg)](https://github.com/mrpt-ros-pkg/mrpt_sensors/actions/workflows/ros-build.yml)
[![CI clang-format](https://github.com/mrpt-ros-pkg/mrpt_sensors/actions/workflows/check-clang-format.yml/badge.svg)](https://github.com/mrpt-ros-pkg/mrpt_sensors/actions/workflows/check-clang-format.yml)


| Distro | Build dev | Release |
| --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mrpt_sensors__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mrpt_sensors__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/humble/mrpt_sensors)](https://index.ros.org/search/?term=mrpt_sensors) |
| ROS 2 Jazzy (u24.04) | [![Build Status](https://build.ros2.org/job/Jdev__mrpt_sensors__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mrpt_sensors__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/jazzy/mrpt_sensors)](https://index.ros.org/search/?term=mrpt_sensors) |
| ROS 2 Kilted (u24.04) | [![Build Status](https://build.ros2.org/job/Kdev__mrpt_sensors__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mrpt_sensors__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/kilted/mrpt_sensors)](https://index.ros.org/search/?term=mrpt_sensors) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mrpt_sensors__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mrpt_sensors__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/rolling/mrpt_sensors)](https://index.ros.org/search/?term=mrpt_sensors) |

| EOL Distro | Last release |
| --- | --- |
| ROS 2 Iron (u22.04) | [![Version](https://img.shields.io/ros/v/iron/mrpt_sensors)](https://index.ros.org/search/?term=mrpt_sensors) |


# mrpt_sensors
ROS nodes for various robotics sensors via mrpt-hwdrivers.

All packages follow [REP-2003](https://ros.org/reps/rep-2003.html) regarding ROS 2 topic QoS.

<!-- md_toc github  < README.md -->

# Table of Contents
- [Diagnostics support](#diagnostics-support)
- [`mrpt_sensor_bumblebee_stereo`](#mrpt_sensor_bumblebee_stereo)
- [`mrpt_sensor_gnss_nmea`](#mrpt_sensor_gnss_nmea)
- [`mrpt_sensor_gnss_novatel`](#mrpt_sensor_gnss_novatel)
- [`mrpt_sensor_imu_taobotics`](#mrpt_sensor_imu_taobotics)
- [`mrpt_sensor_velodyne`](#mrpt_sensor_velodyne)
- [novatel_oem6_msgs](novatel_oem6_msgs/)
- [Individual package build status](#individual-package-build-status)

# Diagnostics support

All sensor nodes in this package publish ROS 2 diagnostics to the `/diagnostics` topic via the
[`diagnostic_updater`](https://github.com/ros/diagnostics) package. The diagnostic status tracks
these conditions:

| Status | Condition |
|--------|-----------|
| `WARN` | Node started but no observation has arrived yet within the startup timeout window |
| `ERROR` | Sensor exception (e.g., serial port unavailable) - node keeps retrying every `retry_on_error_delay` seconds |
| `ERROR` | Sensor was working but no observation has been received for longer than 3x the expected period |
| `WARN` | Observation rate is below 50% of the configured expected rate |
| `OK` | Observations are arriving at the expected rate |

ROS 2 parameters controlling diagnostic thresholds and retry behavior (settable at launch or via `ros2 param set`):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `diag_startup_timeout` | `30.0` s | How long to wait for the first observation before reporting an error |
| `diag_expected_rate` | `1.0` Hz | Expected observation rate; used for stale and rate-warning checks |
| `retry_on_error_delay` | `5.0` s | How long to wait before retrying after a sensor exception |

The diagnostic status also reports the measured observation rate, the observation count, and the
hardware ID (node name).

## Viewing diagnostics

Echo the raw topic directly:

```bash
ros2 topic echo /diagnostics
```

For a human-friendly live view, use `rqt_runtime_monitor` or the `ros2 run` equivalent:

```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

## Running a diagnostic aggregator

The `diagnostic_aggregator` collects and categorizes `/diagnostics` messages into
`/diagnostics_agg`. This is required by tools such as `robot_monitor`. To launch one quickly for
testing, install the package and create a minimal analyzers config:

```bash
sudo apt install ros-${ROS_DISTRO}-diagnostic-aggregator
```

Create a file `analyzers.yaml`:

```yaml
analyzers:
  ros__parameters:
    path: Sensors
    sensors:
      type: diagnostic_aggregator/GenericAnalyzer
      path: Sensors
      contains:
        - 'Sensor status'
```

Then run the aggregator with that config:

```bash
ros2 run diagnostic_aggregator aggregator_node --ros-args \
    --params-file analyzers.yaml
```

In a separate terminal, echo the aggregated output:

```bash
ros2 topic echo /diagnostics_agg
```

Or open `rqt_robot_monitor` for a tree view:

```bash
ros2 run rqt_robot_monitor rqt_robot_monitor
```

---

# `mrpt_sensor_bumblebee_stereo`

ROS 2 node for Bumblebee Stereo Cameras using libdc1394 interface.

Supported models: Bumblebee2 stereo cameras with FireWire connection.

```bash
# INSTALL:
sudo apt install ros-${ROS_DISTRO}-mrpt-sensor-bumblebee-stereo
```

```bash
# Launch to test it:
ros2 launch mrpt_sensor_bumblebee_stereo mrpt_sensor_bumblebee_stereo.launch.py
```

<details>
  <summary>Launch arguments</summary>

```yaml
ros2 launch mrpt_sensor_bumblebee_stereo mrpt_sensor_bumblebee_stereo.launch.py --show-args
Arguments (pass arguments as '<name>:=<value>'):

    'process_rate':
        Rate (Hz) for the process() main sensor loop.
        (default: '"80"')

    'out_rawlog_prefix':
        If not empty, a .rawlog file will be created with all recorded data, apart of publishing it as ROS messages.
        (default: '')

    'publish_mrpt_obs_topic':
        If not empty, mrpt_msgs/GenericObservation messages will be published to this topic name with the binary serialization of mrtp::obs::CObservation objects from the sensor.
        (default: '')

    'publish_topic':
        If not empty, messages of type sensor_msg/Image will be published to this topic (plus suffix "_left"/"_right") for each sensor observation.
        (default: 'sensor')

    'sensor_frame_id':
        The sensor frame_id name. Used to populate msg header and to publish to /tf too.
        (default: 'sensor')

    'robot_frame_id':
        The robot frame_id name. Used to publish the sensor pose to /tf.
        (default: 'base_link')

    'dc1394_framerate':
        eg: 7.5, 15, 30, 60, etc... For possibilities see mrpt::hwdrivers::TCaptureOptions_dc1394
        (default: '"15"')

    'dc1394_camera_guid':
        0 (or not present): the first camera. A hexadecimal number (0x11223344): The GUID of the camera to open
        (default: '"0"')

    'camera_preview_decimation':
        N<=0 (or not present): No preview; N>0, display 1 out of N captured frames.
        (default: '"0"')

    'sensor_label':
        The sensorLabel field of mrpt::obs::CObservation: a "name" for the sensor.
        (default: 'sensor')

    'sensor_pose_x':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'sensor_pose_y':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'sensor_pose_z':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'sensor_pose_yaw':
        Sensor pose coordinate on the vehicle frame (degrees).
        (default: '"0.0"')

    'sensor_pose_pitch':
        Sensor pose coordinate on the vehicle frame (degrees).
        (default: '"0.0"')

    'sensor_pose_roll':
        Sensor pose coordinate on the vehicle frame (degrees).
        (default: '"0.0"')

    'log_level':
        Logging level
        (default: 'INFO')
```
</details>


# `mrpt_sensor_gnss_nmea`

Supported models: any GPS/GNSS sensor emiting NMEA messages via a serial/USB port.

```bash
# INSTALL:
sudo apt install ros-${ROS_DISTRO}-mrpt-sensor-gnss-nmea

```

```bash
# Launch to test it:
ros2 launch mrpt_sensor_gnss_nmea mrpt_sensor_gnss_nmea.launch.py serial_port:=/dev/ttyACM0 publish_topic:="/gps"
```

Parsed messages (via [mrpt::hwdrivers::CGPSInterface](https://docs.mrpt.org/reference/latest/class_mrpt_hwdrivers_CGPSInterface.html)):
- NMEA_GGA
- NMEA_GLL
- NMEA_GSA
- NMEA_GSV
- NMEA_MSS
- NMEA_RMC
- NMEA_VTG
- NMEA_ZDA


<details>
  <summary>Launch arguments</summary>

```yaml
ros2 launch mrpt_sensor_gnss_nmea mrpt_sensor_gnss_nmea.launch.py --show-args
Arguments (pass arguments as '<name>:=<value>'):

    'process_rate':
        Rate (Hz) for the process() main sensor loop.
        (default: '"50"')

    'out_rawlog_prefix':
        If not empty, a .rawlog file will be created with all recorded data, apart of publishing it as ROS messages.
        (default: '')

    'publish_mrpt_obs_topic':
        If not empty, mrpt_msgs/GenericObservation messages will be published to this topic name with the binary serialization of mrtp::obs::CObservation objects from the sensor.
        (default: '')

    'publish_topic':
        If not empty, messages of the appropriate type will be published to this topic for each sensor observation.
        (default: 'sensor')

    'sensor_frame_id':
        The sensor frame_id name. Used to populate msg header and to publish to /tf too.
        (default: 'sensor')

    'sensor_label':
        The sensorLabel field of mrpt::obs::CObservation: a "name" for the sensor.
        (default: 'sensor')

    'serial_port':
        Serial port to open
        (default: '')

    'serial_baud_rate':
        Serial port baud rate (typ: 4800, 9600, etc.)
        (default: '"4800"')

    'sensor_pose_x':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'sensor_pose_y':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'sensor_pose_z':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'log_level':
        Logging level
        (default: 'INFO')
```
</details>


# `mrpt_sensor_gnss_novatel`

ROS node for GNSS/IMU Novatel receivers with RTK precision using an NTRIP HTTP source.
This node actually launches two mrpt::hwdrivers sensors: 

- mrpt::hwdrivers::CGPSInterface for the Novatel GNSS/INS device, and
- mrpt::hwdrivers::CNTRIPEmitter to connect to a remove NTRIP source and emit the received corrections via a serial port.

Thus, you need to specify several mandatory ros launch arguments to make this node to work, 
including the Novatel serial (USB) ports, the NTRIP server configuration, etc.

Supported models: Novatel OEM6

```bash
# INSTALL:
sudo apt install ros-${ROS_DISTRO}-mrpt-sensor-gnss-novatel
```

```bash
# Launch to test it:
ros2 launch mrpt_sensor_gnss_novatel mrpt_sensor_gnss_novatel.launch.py \
    publish_topic:="/gps_novatel" \
    publish_mrpt_obs_topic:="/gps_novatel_mrpt" \
    sensor_frame_id:="novatel" \
    sensor_label:="novatel" \
    novatel_main_serial_port:="/dev/serial/by-id/usb-Novatel_Inc._Novatel_GPS_Receiver_BJYA15400456J-if00-port0" \
    novatel_ntrip_serial_port:="/dev/serial/by-id/usb-Novatel_Inc._Novatel_GPS_Receiver_BJYA15400456J-if00-port2" \
    ntrip_server:="www.euref-ip.net" \
    ntrip_port:="\"2101\"" \
    ntrip_mount_point:="ALME00ESP0" \
    ntrip_user:="user" \
    ntrip_password:="pass" \
    novatel_init_azimuth:="0.0 25.0"

```

<details>
  <summary>Launch arguments</summary>

```yaml
ros2 launch mrpt_sensor_gnss_novatel mrpt_sensor_gnss_novatel.launch.py --show-args
Arguments (pass arguments as '<name>:=<value>'):

    'process_rate':
        Rate (Hz) for the process() main sensor loop.
        (default: '"50"')

    'out_rawlog_prefix':
        If not empty, a .rawlog file will be created with all recorded data, apart of publishing it as ROS messages.
        (default: '')

    'publish_mrpt_obs_topic':
        If not empty, mrpt_msgs/GenericObservation messages will be published to this topic name with the binary serialization of mrtp::obs::CObservation objects from the sensor.
        (default: '')

    'publish_topic':
        If not empty, messages of the appropriate type will be published to this topic for each sensor observation.
        (default: 'sensor')

    'sensor_frame_id':
        The sensor frame_id name. Used to populate msg header and to publish to /tf too.
        (default: 'sensor')

    'robot_frame_id':
        The robot frame_id name. Used to publish the sensor pose to /tf.
        (default: 'base_link')

    'sensor_label':
        The sensorLabel field of mrpt::obs::CObservation: a "name" for the sensor.
        (default: 'sensor')

    'novatel_main_serial_port':
        Main Novatel comms port
        (default: '')

    'serial_baud_rate':
        Serial port baud rate (typ: 4800, 9600, etc.)
        (default: '"4800"')

    'raw_dump_file':
        If not empty, raw GNSS data will be dumped to this file.
        (default: '""')

    'novatel_imu_orientation':
        See Novatel docs for SETIMUORIENTATION.
        (default: '"6"')

    'novatel_veh_body_rotation':
        See Novatel docs for VEHICLEBODYROTATION.
        (default: '"0.000000 0.000000 90.000000 0.000000 0.000000 0.000000"')

    'novatel_imu_to_ant_offset':
        See Novatel docs for SETIMUTOANTOFFSET.
        (default: '"-0.28 -0.08 -0.01 0.000000 0.000000 0.000000"')

    'novatel_ins_offset':
        See Novatel docs for SETINSOFFSET.
        (default: '"0.000000 0.000000 0.000000"')

    'novatel_init_azimuth':
        See Novatel docs for SETINITAZIMUTH.
        (default: '"0.000000 25.000000"')

    'ntrip_server':
        DNS or IP of the NTRIP server.
        (default: '"www.euref-ip.net"')

    'ntrip_port':
        TCP port for connecting to the NTRIP server.
        (default: '"2101"')

    'ntrip_mount_point':
        Mount point to connect inside the NTRIP server.
        (default: '"ALME00ESP0"')

    'ntrip_user':
        NTRIP server username.
        (default: '""')

    'ntrip_password':
        NTRIP server password.
        (default: '""')

    'sensor_pose_x':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'sensor_pose_y':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'sensor_pose_z':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'log_level':
        Logging level
        (default: 'INFO')

```
</details>


<details>
  <summary>Quick Reference: SendNovatelCommand Service</summary>

## Service Call from Command Line

### Basic Syntax
```bash
ros2 service call /gnss/mrpt_sensor_gnss_novatel/send_novatel_command \
    novatel_oem6_msgs/srv/SendNovatelCommand \
    "{command: 'YOUR_COMMAND_HERE'}"
```

## Common Examples

### Set Initial Azimuth (Critical for INS Alignment)
```bash
# Format: SETINITAZIMUTH <azimuth_degrees> <std_dev_degrees>
ros2 service call /gnss/mrpt_sensor_gnss_novatel/send_novatel_command \
    novatel_oem6_msgs/srv/SendNovatelCommand \
    "{command: 'SETINITAZIMUTH 90.0 10.0'}"
```

### Configure IMU Orientation
```bash
# See Novatel manual Table 9 for orientation codes
ros2 service call /gnss/mrpt_sensor_gnss_novatel/send_novatel_command \
    novatel_oem6_msgs/srv/SendNovatelCommand \
    "{command: 'SETIMUORIENTATION 6'}"
```

### INS Control Commands
```bash
# Enable INS
ros2 service call /gnss/mrpt_sensor_gnss_novatel/send_novatel_command \
    novatel_oem6_msgs/srv/SendNovatelCommand \
    "{command: 'INSCOMMAND ENABLE'}"

# Disable INS
ros2 service call /gnss/mrpt_sensor_gnss_novatel/send_novatel_command \
    novatel_oem6_msgs/srv/SendNovatelCommand \
    "{command: 'INSCOMMAND DISABLE'}"

# Reset INS
ros2 service call /gnss/mrpt_sensor_gnss_novatel/send_novatel_command \
    novatel_oem6_msgs/srv/SendNovatelCommand \
    "{command: 'INSCOMMAND RESET'}"
```

### Alignment Mode
```bash
# Set alignment mode (UNAIDED, KINEMATIC, AUTOMATIC, etc.)
ros2 service call /gnss/mrpt_sensor_gnss_novatel/send_novatel_command \
    novatel_oem6_msgs/srv/SendNovatelCommand \
    "{command: 'ALIGNMENTMODE UNAIDED'}"
```

### Satellite Configuration
```bash
# Set elevation mask (degrees above horizon)
ros2 service call /gnss/mrpt_sensor_gnss_novatel/send_novatel_command \
    novatel_oem6_msgs/srv/SendNovatelCommand \
    "{command: 'ECUTOFF 10'}"

# Set C/N0 mask (signal strength threshold)
ros2 service call /gnss/mrpt_sensor_gnss_novatel/send_novatel_command \
    novatel_oem6_msgs/srv/SendNovatelCommand \
    "{command: 'CNOECUTOFF 25'}"
```

## Notes

1. **CR+LF Added Automatically**: You don't need to add `\r\n` - the service does this
2. **Command Case**: Commands are case-insensitive in Novatel firmware
3. **Service Namespace**: Default is `/gnss/mrpt_sensor_gnss_novatel/`
4. **Response**: Service returns `done_ok: true` if command was sent (not if it succeeded on receiver)
5. **Verification**: Check receiver response by monitoring the node's log output or relevant message topics

</details>

# `mrpt_sensor_imu_taobotics`

Supported models: `hfi-b6`, `hfi-a9`

```bash
# INSTALL:
sudo apt install ros-${ROS_DISTRO}-mrpt-sensor-imu-taobotics
```

```bash
# Launch to test it:
ros2 launch mrpt_sensor_imu_taobotics mrpt_sensor_imu_taobotics.launch.py serial_port:=/dev/ttyUSB0
```

<details>
  <summary>Launch arguments</summary>

```yaml
ros2 launch mrpt_sensor_imu_taobotics mrpt_sensor_imu_taobotics.launch.py --show-args
Arguments (pass arguments as '<name>:=<value>'):

    'process_rate':
        Rate (Hz) for the process() main sensor loop.
        (default: '"500"')

    'out_rawlog_prefix':
        If not empty, a .rawlog file will be created with all recorded data, apart of publishing it as ROS messages.
        (default: '')

    'publish_mrpt_obs_topic':
        If not empty, mrpt_msgs/GenericObservation messages will be published to this topic name with the binary serialization of mrtp::obs::CObservation objects from the sensor.
        (default: '')

    'publish_topic':
        If not empty, messages of type sensor_msg/Image will be published to this topic (plus suffix "_left"/"_right") for each sensor observation.
        (default: 'sensor')

    'sensor_frame_id':
        The sensor frame_id name. Used to populate msg header and to publish to /tf too.
        (default: 'imu')

    'robot_frame_id':
        The robot frame_id name. Used to publish the sensor pose to /tf.
        (default: 'base_link')

    'serial_port':
        Serial port device to open, e.g. /dev/ttyUSB0

    'sensor_model':
        Sensor model, needed to parse its binary frame protocol. Supported devices (check mrpt::hwdrivers::CTaoboticsIMU) at present are: (hfi-b6|hfi-a9)
        (default: '"hfi-a9"')

    'sensor_label':
        The sensorLabel field of mrpt::obs::CObservation: a "name" for the sensor.
        (default: 'sensor')

    'sensor_pose_x':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'sensor_pose_y':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'sensor_pose_z':
        Sensor pose coordinate on the vehicle frame.
        (default: '"0.0"')

    'sensor_pose_yaw':
        Sensor pose coordinate on the vehicle frame (degrees).
        (default: '"0.0"')

    'sensor_pose_pitch':
        Sensor pose coordinate on the vehicle frame (degrees).
        (default: '"0.0"')

    'sensor_pose_roll':
        Sensor pose coordinate on the vehicle frame (degrees).
        (default: '"0.0"')

    'log_level':
        Logging level
        (default: 'INFO')

```
</details>


# `mrpt_sensor_velodyne`

Supported models: `VLP16`,`HDL32`,`HDL64`

```bash
ros2 launch mrpt_sensors velodyne.launch.py
```

TODO: Document and explain parameters.


# Individual package build status

| Package | ROS 2 Humble <br/> BinBuild | ROS 2 Jazzy <br/> BinBuild | ROS 2 Kilted <br/> BinBuild |  ROS 2 Rolling <br/> BinBuild |
| --- | --- | --- | --- |--- |
| mrpt_generic_sensor | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_generic_sensor__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_generic_sensor__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_generic_sensor__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_generic_sensor__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_generic_sensor__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_generic_sensor__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_generic_sensor__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_generic_sensor__ubuntu_noble_amd64__binary/) |
| mrpt_sensor_bumblebee_stereo | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensor_bumblebee_stereo__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensor_bumblebee_stereo__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_sensor_bumblebee_stereo__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_sensor_bumblebee_stereo__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_sensor_bumblebee_stereo__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_sensor_bumblebee_stereo__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_sensor_bumblebee_stereo__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_sensor_bumblebee_stereo__ubuntu_noble_amd64__binary/) |
| mrpt_sensor_gnss_nmea | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensor_gnss_nmea__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensor_gnss_nmea__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_sensor_gnss_nmea__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_sensor_gnss_nmea__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_sensor_gnss_nmea__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_sensor_gnss_nmea__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_sensor_gnss_nmea__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_sensor_gnss_nmea__ubuntu_noble_amd64__binary/) |
| mrpt_sensor_gnss_novatel | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensor_gnss_novatel__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensor_gnss_novatel__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_sensor_gnss_novatel__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_sensor_gnss_novatel__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_sensor_gnss_novatel__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_sensor_gnss_novatel__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_sensor_gnss_novatel__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_sensor_gnss_novatel__ubuntu_noble_amd64__binary/) |
| mrpt_sensor_imu_taobotics | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensor_imu_taobotics__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensor_imu_taobotics__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_sensor_imu_taobotics__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_sensor_imu_taobotics__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_sensor_imu_taobotics__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_sensor_imu_taobotics__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_sensor_imu_taobotics__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_sensor_imu_taobotics__ubuntu_noble_amd64__binary/) |
| mrpt_sensorlib | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensorlib__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensorlib__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_sensorlib__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_sensorlib__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_sensorlib__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_sensorlib__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_sensorlib__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_sensorlib__ubuntu_noble_amd64__binary/) |
| mrpt_sensors | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensors__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_sensors__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_sensors__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_sensors__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_sensors__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_sensors__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_sensors__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_sensors__ubuntu_noble_amd64__binary/) |
| novatel_oem6_msgs | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__novatel_oem6_msgs__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__novatel_oem6_msgs__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__novatel_oem6_msgs__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__novatel_oem6_msgs__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__novatel_oem6_msgs__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__novatel_oem6_msgs__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__novatel_oem6_msgs__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__novatel_oem6_msgs__ubuntu_noble_amd64__binary/) |
