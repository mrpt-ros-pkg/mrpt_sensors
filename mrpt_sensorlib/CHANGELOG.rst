^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_sensorlib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.2.3 (2024-09-01)
------------------
* Update copyright comment blocks to try to make ament linter happy
* Depend on new mrpt_lib packages (deprecate mrpt2)
* Update to package XML format 3
* Contributors: Jose Luis Blanco-Claraco

0.2.2 (2024-08-09)
------------------
* Merge pull request `#10 <https://github.com/mrpt-ros-pkg/mrpt_sensors/issues/10>`_ from mrpt-ros-pkg/use-gnss-base-topic-name
  Use gnss base topic name
* Reuse base class publish topic name
  add missing getter
* Contributors: Jose Luis Blanco-Claraco

0.2.1 (2024-07-03)
------------------
* Publish NMEA ROS2 native msgs: GGA, GSA, RMC
* Contributors: Jose Luis Blanco-Claraco

0.2.0 (2024-05-20)
------------------
* FIX: Implement the missing "saveToRawlog" feature
* Add parameter to set the sensorLabel of generated observations
* Limit publication of /tf sensor poses to a maximum configurable rate
* BUGFIX: tf error if sensor_frame_id==robot_frame_id
* Contributors: Jose Luis Blanco-Claraco

0.1.0 (2024-03-28)
------------------
* publish sensor pose to /tf
* fix missing namespace
* Reformat with clang-format to fix ament_linters
* Comply with ROS2 REP-2003
* Fix usage of obsolete mrpt methods
* delegate conversion to mrpt::ros2bridge
* Port to ROS2
* Contributors: Jose Luis Blanco-Claraco
