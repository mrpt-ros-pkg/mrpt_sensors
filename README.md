| Distro | Build dev | Build release | Stable release |
| --- | --- | --- | --- |
| ROS2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mrpt_sensors__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mrpt_sensors__ubuntu_jammy_amd64/) |  xxx | xxx |
| ROS2 Iron (u22.04) | [![Build Status](https://build.ros2.org/job/Idev__mrpt_sensors__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Idev__mrpt_sensors__ubuntu_jammy_amd64/) |  xxx | xxx |
| ROS2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mrpt_sensors__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mrpt_sensors__ubuntu_jammy_amd64/) |  xxx | xxx |

# mrpt_sensors
ROS nodes for various robotics sensors via mrpt-hwdrivers.

All packages follow [REP-2003](https://ros.org/reps/rep-2003.html) regarding ROS 2 topic QoS.

<!-- toc -->

- [`mrpt_sensor_gnns_nmea`](#mrpt_sensor_gnns_nmea)
  * [Usage](#usage)
  * [Launch arguments](#launch-arguments)
- [`mrpt_sensor_velodyne`](#mrpt_sensor_velodyne)

<!-- tocstop -->

# `mrpt_sensor_gnns_nmea`

## Usage
Supported models: any GPS/GNNS sensor emiting NMEA messages via a serial/USB port.

```bash
ros2 launch mrpt_sensor_gnns_nmea mrpt_sensor_gnns_nmea.launch.py serial_port:=/dev/ttyACM0 publish_topic:="/gps"
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

## Launch arguments

```yaml
ros2 launch mrpt_sensor_gnns_nmea mrpt_sensor_gnns_nmea.launch.py --show-args
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

# `mrpt_sensor_velodyne`

Supported models: `VLP16`,`HDL32`,`HDL64`

```bash
ros2 launch mrpt_sensors velodyne.launch.py
```

TODO: Document and explain parameters.
