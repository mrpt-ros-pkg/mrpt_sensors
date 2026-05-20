^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_sensor_gnss_novatel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.0 (2026-05-20)
------------------
* Publish oem6 ros2 messages too
* Add novatel_oem6_msgs package and service to send custom commands to Novatel
* Fix param type errors
* Fix obsolete tf headers
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

0.2.1 (2024-07-03)
------------------