^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_sensor_gnss_novatel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.4.1 (2026-10-27)
------------------
* Merge pull request `#22 <https://github.com/mrpt-ros-pkg/mrpt_sensors/issues/22>`_ from mrpt-ros-pkg/feat/novatel-rangecmp
  Novatel: publish per-signal tracking status (decoded RANGECMP)
* Novatel: publish per-signal tracking status (decoded RANGECMP) on ~/novatel/rangecmp
  New messages novatel_oem6_msgs/RANGECMP and RangeObs: for each tracked
  signal, satellite system, PRN, signal type, C/No, lock time, pseudorange,
  Doppler and lock flags. RANGECMPB was already logged at 1 Hz.
* Merge pull request `#19 <https://github.com/mrpt-ros-pkg/mrpt_sensors/issues/19>`_ from mrpt-ros-pkg/fix/novatel-ntrip-robustness
  Novatel: robust NTRIP corrections, per-sensor retries and NTRIP diagnostics
* Accept integer NTRIP time parameters; better one-line error messages
  - ntrip_stream_timeout and ntrip_reconnect_delay accept both integer and
  floating point values.
  - short_error_message(): without a 'Message:' tag, use the first line
  other than the MRPT exception banner.
* Novatel: robust NTRIP corrections with diagnostics
  Replace CNTRIPEmitter by an NTRIP forwarder running in its own thread,
  which recovers by itself from:
  - the caster not being reachable at start up (it was never retried),
  - silent link losses: reconnects after ntrip_stream_timeout seconds
  without data (the TCP connection could stall forever),
  - USB disconnections of the corrections serial port (never reopened).
  It also sends the receiver GGA sentences to the caster (ntrip_send_gga),
  needed by VRS mount points, and no longer delays the GNSS data reading.
  New 'NTRIP status' diagnostics entry: connection and serial port state,
  data rate, time since last data, connection count, last errors, and the
  correction age and base station ID reported by the receiver in BESTPOS.
  New launch arguments: ntrip_stream_timeout, ntrip_reconnect_delay,
  ntrip_send_gga. An empty novatel_ntrip_serial_port disables NTRIP.
* Merge pull request `#18 <https://github.com/mrpt-ros-pkg/mrpt_sensors/issues/18>`_ from mrpt-ros-pkg/fix/novatel-no-generic-frames
  Novatel: do not request BESTVELB and INSATTB logs
* Novatel: do not request BESTVELB and INSATTB logs
  They have no dedicated MRPT parser and go through the generic frame
  container, whose decoding reads a wrong length and allocates gigabytes,
  stalling the node as soon as the receiver outputs them (outdoors).
* Merge pull request `#16 <https://github.com/mrpt-ros-pkg/mrpt_sensors/issues/16>`_ from mrpt-ros-pkg/fix/novatel-base-station-id-nulls
  Novatel: fix crash publishing BESTPOS/MARKPOS without a base station
* Novatel: fix crash publishing BESTPOS/MARKPOS without a base station
  The 4-char base_station_id was copied including its NUL padding, and
  Fast-CDR refuses to serialize strings with embedded NULs: publish() threw,
  the exception escaped run() and the node died (SIGSEGV while unwinding).
  - Novatel: keep only up to the first NUL of fixed-size char fields.
  - Novatel: build the header with create_header(), which falls back to the
  current time for invalid GNSS stamps (as the NMEA node already does).
  - sensorlib: an exception while processing one observation is logged
  instead of killing the node.
  - sensorlib: stop the loop when the ROS context is shut down, so CTRL+C
  does not end with 'context is not valid' errors.
* Contributors: Jose Luis Blanco-Claraco

0.4.0 (2026-10-01)
------------------
* Merge pull request `#13 <https://github.com/mrpt-ros-pkg/mrpt_sensors/issues/13>`_ from mrpt-ros-pkg/mrpt3
* package.xml: rename mrpt_libhwdrivers dep to mrpt_hwdrivers
* fix(novatel): use the config key names CGPSInterface actually reads
* Port to mrpt3: fix cmake targets, yaml API, and mrpt3 API changes
* Contributors: Jose Luis Blanco-Claraco

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
