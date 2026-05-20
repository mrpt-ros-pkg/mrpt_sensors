^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_sensorlib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-05-20)
------------------
* feat(mrpt_sensorlib): retry on sensor exception instead of crashing
  When doProcess() throws (e.g. serial port unavailable at startup),
  catch the exception, report ERROR via /diagnostics, and retry after
  retry_on_error_delay seconds (default 5 s, tunable via ROS 2 param).
* feat(mrpt_sensorlib): add /diagnostics support to all sensor nodes
  Centralised in GenericSensorNode so every sensor package inherits it
  automatically without any per-package changes.
  State machine:
  - WARN "initialising" for the first `diag_startup_timeout` seconds (default 30 s)
  - ERROR if no observation received after that timeout
  - ERROR if time since last observation exceeds 3× the expected period (stale)
  - WARN if smoothed rate < 50% of expected rate
  - OK otherwise
  New ROS 2 parameters (both in mrpt_sensorlib):
  diag_startup_timeout  [s]  (default 30.0)
  diag_expected_rate    [Hz] (default 1.0)
* Novatel SPAN node can now subscribe to an IMU and use its orientation to initialize the INS azimuth
* FIX: Don't throw if an invalid stamp arrives from the sensor
* Contributors: Jose Luis Blanco-Claraco

0.2.4 (2025-10-27)
------------------
* Replace deprecated ament_target_dependencies() with standard cmake
* package.xml: update license tag to BSD-3-Clause
* Contributors: Jose Luis Blanco-Claraco

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
