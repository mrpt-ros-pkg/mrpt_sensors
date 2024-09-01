^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_sensor_gnss_nmea
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* Merge pull request `#8 <https://github.com/mrpt-ros-pkg/mrpt_sensors/issues/8>`_ from mrpt-ros-pkg/use-gnss-base-topic-name
  Reuse base class publish topic name
* Reuse base class publish topic name
* Merge pull request `#6 <https://github.com/mrpt-ros-pkg/mrpt_sensors/issues/6>`_ from r-aguilera/ros2
  fix uninitialized publishers in NMEA msg publishers
* fix uninitialized publishers
* Contributors: Jose Luis Blanco-Claraco, Raúl Aguilera

0.2.1 (2024-07-03)
------------------
* Fix typo in GNSS names
* Contributors: Jose Luis Blanco-Claraco

0.2.0 (2024-05-20)
------------------
* Add parameter to set the sensorLabel of generated observations
* Contributors: Jose Luis Blanco-Claraco

0.1.0 (2024-03-28)
------------------
* publish sensor pose to /tf
* Reformat with clang-format to fix ament_linters
* more conservative uncertainty
* fix covariance term index
* create gnss node
* Contributors: Jose Luis Blanco-Claraco
