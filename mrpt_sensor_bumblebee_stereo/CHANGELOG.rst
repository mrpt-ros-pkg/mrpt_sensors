^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_sensor_bumblebee_stereo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Lossless raw Bayer images by default (bayer_decoding:=true publishes rgb8 instead).
* Topics follow the ROS camera conventions (left/image_raw, right/image_raw), plus low-rate topics (low_rate_hz, default 1 Hz).
* Both images of a pair share the capture stamp; optical frames published to /tf_static (stereo_baseline).
* Launch: namespace and node_name arguments.
* Publish the sensor pose to /tf_static instead of periodically to /tf.
* Contributors: Jose Luis Blanco-Claraco

0.4.1 (2026-10-27)
------------------

0.4.0 (2026-10-01)
------------------
* Merge pull request `#13 <https://github.com/mrpt-ros-pkg/mrpt_sensors/issues/13>`_ from mrpt-ros-pkg/mrpt3
* package.xml: rename mrpt_libhwdrivers dep to mrpt_hwdrivers
* Port to mrpt3: fix cmake targets, yaml API, and mrpt3 API changes
* Contributors: Jose Luis Blanco-Claraco

0.3.0 (2026-05-20)
------------------
* remove obsolete dependencies
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
* Add new driver for Novatel OEM6 + NTRIP server
* Contributors: Jose Luis Blanco-Claraco

0.2.0 (2024-05-20)
------------------
* Add parameter to set the sensorLabel of generated observations
* BUGFIX: Wrong parsing of sensor "roll" angle from ros2 params
* add XSens IMU sensor driver wrapper
* Contributors: Jose Luis Blanco-Claraco

0.1.0 (2024-03-28)
------------------
* publish sensor pose to /tf
* Reformat with clang-format to fix ament_linters
* Add driver for Bumblebee stereo camera
* Contributors: Jose Luis Blanco-Claraco
