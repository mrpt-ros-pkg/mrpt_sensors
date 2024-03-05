| Distro | Build dev | Build release | Stable release |
| --- | --- | --- | --- |
| ROS2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mrpt_sensors__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mrpt_sensors__ubuntu_jammy_amd64/) |  xxx | xxx |
| ROS2 Iron (u22.04) | [![Build Status](https://build.ros2.org/job/Idev__mrpt_sensors__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Idev__mrpt_sensors__ubuntu_jammy_amd64/) |  xxx | xxx |
| ROS2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mrpt_sensors__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mrpt_sensors__ubuntu_jammy_amd64/) |  xxx | xxx |

# mrpt_sensors
ROS nodes for various robotics sensors via mrpt-hwdrivers

# Usage

## Velodyne
Supported models: `VLP16`,`HDL32`,`HDL64`

```bash
ros2 launch mrpt_sensors velodyne.launch.py
```

TODO: Document and explain parameters.

## NMEA GNNS
Supported models: any GPS/GNNS sensor emiting NMEA messages via a serial/USB port.

```bash
ros2 launch mrpt_sensors nmea_gnns.launch.py
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

TODO: Document and explain parameters.

