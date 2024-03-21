| Distro | Build dev | Build release | Stable release |
| --- | --- | --- | --- |
| ROS2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mrpt_sensors__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mrpt_sensors__ubuntu_jammy_amd64/) |  xxx | xxx |
| ROS2 Iron (u22.04) | [![Build Status](https://build.ros2.org/job/Idev__mrpt_sensors__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Idev__mrpt_sensors__ubuntu_jammy_amd64/) |  xxx | xxx |
| ROS2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mrpt_sensors__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mrpt_sensors__ubuntu_jammy_amd64/) |  xxx | xxx |

# mrpt_sensors
ROS nodes for various robotics sensors via mrpt-hwdrivers.

All packages follow [REP-2003](https://ros.org/reps/rep-2003.html) regarding ROS 2 topic QoS.

# Table of Contents
* [mrpt_sensors](#mrpt_sensors)
* [`mrpt_sensor_bumblebee_stereo`](#`mrpt_sensor_bumblebee_stereo`)
	* [Usage](#Usage)
	* [Launch arguments](#Launch-arguments)
* [`mrpt_sensor_gnns_nmea`](#`mrpt_sensor_gnns_nmea`)
	* [Usage](#Usage)
	* [Launch arguments](#Launch-arguments)
* [`mrpt_sensor_velodyne`](#`mrpt_sensor_velodyne`)


# `mrpt_sensor_bumblebee_stereo`

ROS 2 node for Bumblebee Stereo Cameras using libdc1394 interface.

## Usage
Supported models: Bumblebee2 stereo cameras with FireWire connection.

```bash
ros2 launch mrpt_sensor_bumblebee_stereo mrpt_sensor_bumblebee_stereo.launch.py
```

## Launch arguments

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
