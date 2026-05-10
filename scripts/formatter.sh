#!/usr/bin/env bash
# Usage: scripts/formatter.sh [--check]
#   (default) Reformat all C/C++ sources in-place with clang-format-14.
#   --check   Dry-run: exit non-zero if any file would be reformatted.

set -euo pipefail

if [ "${1:-}" = "--check" ]; then
  MODE=(--dry-run --Werror)
else
  MODE=(-i)
fi

find \
    mrpt_generic_sensor \
    mrpt_sensor_bumblebee_stereo \
    mrpt_sensor_gnss_nmea \
    mrpt_sensor_gnss_novatel \
    mrpt_sensor_imu_taobotics \
    mrpt_sensorlib \
    mrpt_sensors \
    \( -iname "*.h" -o -iname "*.hpp" -o -iname "*.cpp" -o -iname "*.c" \) \
  -print0 | xargs -0 -r -t clang-format-14 "${MODE[@]}"
