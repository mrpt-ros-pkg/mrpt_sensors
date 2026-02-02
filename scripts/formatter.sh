#!/bin/bash

# formatter.sh
find  \
    mrpt_generic_sensor \
    mrpt_sensor_bumblebee_stereo \
    mrpt_sensor_gnss_nmea \
    mrpt_sensor_gnss_novatel \
    mrpt_sensor_imu_taobotics \
    mrpt_sensorlib \
    mrpt_sensors \
    -iname *.h -o -iname *.hpp -o -iname *.cpp -o -iname *.c | xargs clang-format-14 -i
