^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package novatel_oem6_msgs
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2026-10-27)
------------------
* Merge pull request `#22 <https://github.com/mrpt-ros-pkg/mrpt_sensors/issues/22>`_ from mrpt-ros-pkg/feat/novatel-rangecmp
  Novatel: publish per-signal tracking status (decoded RANGECMP)
* Novatel: publish per-signal tracking status (decoded RANGECMP) on ~/novatel/rangecmp
  New messages novatel_oem6_msgs/RANGECMP and RangeObs: for each tracked
  signal, satellite system, PRN, signal type, C/No, lock time, pseudorange,
  Doppler and lock flags. RANGECMPB was already logged at 1 Hz.
* Contributors: Jose Luis Blanco-Claraco

0.4.0 (2026-10-01)
------------------

0.3.0 (2026-05-20)
------------------
* Publish oem6 ros2 messages too
* Add novatel_oem6_msgs package and service to send custom commands to Novatel
* Contributors: Jose Luis Blanco-Claraco

0.2.4 (2025-10-27)
------------------
* Contributors: Jose Luis Blanco-Claraco
