/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   |                                                                        |
   | Redistribution and use in source and binary forms, with or without     |
   | modification, are permitted provided that the following conditions     |
   | are met:                                                               |
   |                                                                        |
   | 1. Redistributions of source code must retain the above copyright      |
   |    notice, this list of conditions and the following disclaimer.       |
   |                                                                        |
   | 2. Redistributions in binary form must reproduce the above copyright   |
   |    notice, this list of conditions and the following disclaimer in the |
   |    documentation and/or other materials provided with the distribution.|
   |                                                                        |
   | 3. Neither the name of the copyright holder nor the names of its       |
   |    contributors may be used to endorse or promote products derived     |
   |    from this software without specific prior written permission.       |
   |                                                                        |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    |
   | "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      |
   | LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  |
   | A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   |
   | HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |
   | SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       |
   | LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  |
   | DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  |
   | THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    |
   | (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  |
   | OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   |
   |                                                                        |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   | SPDX-License-Identifier: BSD-3-Clause                                  |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/Clock.h>
#include <mrpt/core/bits_math.h>  // square()
#include <mrpt/core/get_env.h>
#include <mrpt/hwdrivers/CGPSInterface.h>
#include <mrpt/math/wrap2pi.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/obs/gnss_messages_novatel.h>
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/time.h>
#include <mrpt/system/COutputLogger.h>

#include <cmath>
#include <mutex>
#include <novatel_oem6_msgs/msg/bestpos.hpp>
#include <novatel_oem6_msgs/msg/detail/inscovs__struct.hpp>
#include <novatel_oem6_msgs/msg/detail/inspvas__struct.hpp>
#include <novatel_oem6_msgs/msg/detail/ionutc__struct.hpp>
#include <novatel_oem6_msgs/msg/detail/mark2_time__struct.hpp>
#include <novatel_oem6_msgs/msg/detail/markpos__struct.hpp>
#include <novatel_oem6_msgs/msg/detail/marktime__struct.hpp>
#include <novatel_oem6_msgs/msg/detail/rawephem__struct.hpp>
#include <novatel_oem6_msgs/msg/detail/rawimus__struct.hpp>
#include <novatel_oem6_msgs/msg/detail/rxstatus__struct.hpp>
#include <novatel_oem6_msgs/msg/inscovs.hpp>
#include <novatel_oem6_msgs/msg/inspvas.hpp>
#include <novatel_oem6_msgs/msg/ionutc.hpp>
#include <novatel_oem6_msgs/msg/mark2_time.hpp>
#include <novatel_oem6_msgs/msg/markpos.hpp>
#include <novatel_oem6_msgs/msg/marktime.hpp>
#include <novatel_oem6_msgs/msg/rawephem.hpp>
#include <novatel_oem6_msgs/msg/rawimus.hpp>
#include <novatel_oem6_msgs/msg/rxstatus.hpp>
#include <novatel_oem6_msgs/srv/send_novatel_command.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/nav_sat_fix__struct.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Quaternion.hpp>

#include "mrpt_sensorlib/mrpt_sensorlib.h"

namespace
{
const char* node_name = "mrpt_sensor_gnss_novatel";

const bool VERBOSE = mrpt::get_env<bool>("VERBOSE");

// INS status values from Novatel (see OEM6/OEM7 manual)
// These correspond to the ins_status field in INSPVAS messages
enum class InsStatus : uint32_t
{
  INS_INACTIVE = 0,
  INS_ALIGNING = 1,
  INS_HIGH_VARIANCE = 2,
  INS_SOLUTION_GOOD = 3,
  INS_SOLUTION_FREE = 6,
  INS_ALIGNMENT_COMPLETE = 7,
  INS_DETERMINING_ORIENTATION = 8,
  INS_WAITING_INITIALPOS = 9,
  INS_WAITING_AZIMUTH = 10,
  INS_INITIALIZING_BIASES = 11,
  INS_MOTION_DETECT = 12
};

const char* insStatusToString(InsStatus status)
{
  switch (status)
  {
    case InsStatus::INS_INACTIVE:
      return "INS_INACTIVE";
    case InsStatus::INS_ALIGNING:
      return "INS_ALIGNING";
    case InsStatus::INS_HIGH_VARIANCE:
      return "INS_HIGH_VARIANCE";
    case InsStatus::INS_SOLUTION_GOOD:
      return "INS_SOLUTION_GOOD";
    case InsStatus::INS_SOLUTION_FREE:
      return "INS_SOLUTION_FREE";
    case InsStatus::INS_ALIGNMENT_COMPLETE:
      return "INS_ALIGNMENT_COMPLETE";
    case InsStatus::INS_DETERMINING_ORIENTATION:
      return "INS_DETERMINING_ORIENTATION";
    case InsStatus::INS_WAITING_INITIALPOS:
      return "INS_WAITING_INITIALPOS";
    case InsStatus::INS_WAITING_AZIMUTH:
      return "INS_WAITING_AZIMUTH";
    case InsStatus::INS_INITIALIZING_BIASES:
      return "INS_INITIALIZING_BIASES";
    case InsStatus::INS_MOTION_DETECT:
      return "INS_MOTION_DETECT";
    default:
      return "UNKNOWN";
  }
}

// Structure to hold external IMU data
struct ExternalImuData
{
  double azimuth_deg = 0.0;  // Heading/yaw in degrees (0-360, CW from North)
  double azimuth_std_deg = 25.0;  // Standard deviation in degrees
  rclcpp::Time timestamp;
  bool valid = false;
};

// Node state for managing INS alignment
struct NovatelNodeState
{
  std::mutex mutex;
  InsStatus last_ins_status = InsStatus::INS_INACTIVE;
  bool azimuth_sent = false;
  ExternalImuData external_imu;
  mrpt::hwdrivers::CGPSInterface* gps_interface = nullptr;

  // Parameters
  bool use_external_imu_azimuth = false;
  double external_imu_max_age_sec = 2.0;
  double external_imu_azimuth_deg_for_yaw_zero = 0.0;
  double manual_init_azimuth_deg = 0.0;
  double manual_init_azimuth_std_deg = 10.0;
};

// Convert quaternion to azimuth (heading) in degrees
// Azimuth for Novatel SPAN is:
// Left-handed rotation around z-axis in degrees clockwise from North.
// Assumes the IMU provides orientation in ENU frame with yaw=0 being North
// Output: azimuth in degrees, 0-360, clockwise from North
double quaternionToAzimuthDeg(
    const sensor_msgs::msg::Imu& imu_msg, double imu_azimuth_deg_for_yaw_zero)
{
  tf2::Quaternion q(
      imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Convert from ENU yaw (rad, CCW from East) to azimuth (deg, CW from North)
  // ENU: yaw=0 is East, positive CCW
  // Azimuth: 0 is North, positive CW
  // Azimuth = imu_azimuth_deg_for_yaw_zero (e.g.90) - yaw_deg
  double yaw_deg = yaw * 180.0 / M_PI;
  double azimuth = imu_azimuth_deg_for_yaw_zero - yaw_deg;

  // Wrap to [0, 360)
  while (azimuth < 0.0)
  {
    azimuth += 360.0;
  }
  while (azimuth >= 360.0)
  {
    azimuth -= 360.0;
  }

  return azimuth;
}

// Send SETINITAZIMUTH command to Novatel
bool sendAzimuthToNovatel(
    NovatelNodeState& state, double azimuth_deg, double std_deg, rclcpp::Logger& logger)
{
  if (!state.gps_interface)
  {
    RCLCPP_ERROR(logger, "GPS interface not available for sending commands");
    return false;
  }

  // Format: SETINITAZIMUTH azimuth stddev
  // azimuth in degrees (0-360, CW from North)
  // stddev in degrees (1 sigma)
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "SETINITAZIMUTH %.6f %.6f\r\n", azimuth_deg, std_deg);

  RCLCPP_INFO(logger, "Sending azimuth command: SETINITAZIMUTH %.2f %.2f", azimuth_deg, std_deg);

  return state.gps_interface->sendCustomCommand(cmd, strlen(cmd));
}

// Check INS status and send azimuth if needed
void checkAndSendAzimuth(NovatelNodeState& state, rclcpp::Node& node, rclcpp::Logger& logger)
{
  std::lock_guard<std::mutex> lock(state.mutex);

  // Only send azimuth when INS is waiting for it or stuck in aligning
  if (state.last_ins_status != InsStatus::INS_WAITING_AZIMUTH &&
      state.last_ins_status != InsStatus::INS_ALIGNING)
  {
    return;
  }

  if (state.azimuth_sent)
  {
    return;
  }

  double azimuth_deg = state.manual_init_azimuth_deg;
  double azimuth_std_deg = state.manual_init_azimuth_std_deg;

  // Prefer external IMU if enabled and data is fresh
  if (state.use_external_imu_azimuth && state.external_imu.valid)
  {
    double age = (node.get_clock()->now() - state.external_imu.timestamp).seconds();

    if (age <= state.external_imu_max_age_sec)
    {
      azimuth_deg = state.external_imu.azimuth_deg;
      azimuth_std_deg = state.external_imu.azimuth_std_deg;

      RCLCPP_INFO(
          logger,
          "Using external IMU azimuth: %.2f deg (std: %.2f deg, age: "
          "%.2f s)",
          azimuth_deg, azimuth_std_deg, age);
    }
    else
    {
      RCLCPP_WARN(
          logger,
          "External IMU data too old (%.2f s > %.2f s), using manual "
          "azimuth: %.2f deg",
          age, state.external_imu_max_age_sec, azimuth_deg);
    }
  }

  if (sendAzimuthToNovatel(state, azimuth_deg, azimuth_std_deg, logger))
  {
    state.azimuth_sent = true;
    RCLCPP_INFO(
        logger, "Azimuth command sent: %.2f deg (std: %.2f deg)", azimuth_deg, azimuth_std_deg);
  }
  else
  {
    RCLCPP_ERROR(logger, "Failed to send azimuth command");
  }
}

struct NovatelPublishers
{
  rclcpp::Publisher<novatel_oem6_msgs::msg::BESTPOS>::SharedPtr bestpos;
  rclcpp::Publisher<novatel_oem6_msgs::msg::INSPVAS>::SharedPtr inspvas;
  rclcpp::Publisher<novatel_oem6_msgs::msg::INSCOVS>::SharedPtr inscovs;
  rclcpp::Publisher<novatel_oem6_msgs::msg::RAWIMUS>::SharedPtr rawimus;
  rclcpp::Publisher<novatel_oem6_msgs::msg::MARKPOS>::SharedPtr markpos;
  rclcpp::Publisher<novatel_oem6_msgs::msg::MARKTIME>::SharedPtr marktime;
  rclcpp::Publisher<novatel_oem6_msgs::msg::MARK2TIME>::SharedPtr mark2time;
  rclcpp::Publisher<novatel_oem6_msgs::msg::IONUTC>::SharedPtr ionutc;
  rclcpp::Publisher<novatel_oem6_msgs::msg::RAWEPHEM>::SharedPtr rawephem;
  rclcpp::Publisher<novatel_oem6_msgs::msg::RXSTATUS>::SharedPtr rxstatus;
};

template <typename T>
void ensurePublisher(
    typename rclcpp::Publisher<T>::SharedPtr& pub, rclcpp::Node* node, const std::string& topic)
{
  if (!pub)
  {
    pub = node->create_publisher<T>(topic, rclcpp::SensorDataQoS());
  }
}

// ============================================================================
// CONVERSION FUNCTIONS (add to anonymous namespace)
// ============================================================================

novatel_oem6_msgs::msg::BESTPOS convertBESTPOS(
    const mrpt::obs::gnss::Message_NV_OEM6_BESTPOS& msg, const std_msgs::msg::Header& header)
{
  novatel_oem6_msgs::msg::BESTPOS out;
  out.header = header;

  out.solution_stat = msg.fields.solution_stat;
  out.position_type = msg.fields.position_type;
  out.lat = msg.fields.lat;
  out.lon = msg.fields.lon;
  out.hgt = msg.fields.hgt;
  out.undulation = msg.fields.undulation;
  out.datum_id = msg.fields.datum_id;
  out.lat_sigma = msg.fields.lat_sigma;
  out.lon_sigma = msg.fields.lon_sigma;
  out.hgt_sigma = msg.fields.hgt_sigma;
  out.base_station_id = std::string(msg.fields.base_station_id, 4);
  out.diff_age = msg.fields.diff_age;
  out.sol_age = msg.fields.sol_age;
  out.num_sats_tracked = msg.fields.num_sats_tracked;
  out.num_sats_sol = msg.fields.num_sats_sol;
  out.num_sats_sol_l1 = msg.fields.num_sats_sol_L1;
  out.num_sats_sol_multi = msg.fields.num_sats_sol_multi;
  out.reserved = msg.fields.reserved;
  out.ext_sol_stat = msg.fields.ext_sol_stat;
  out.galileo_beidou_mask = msg.fields.galileo_beidou_mask;
  out.gps_glonass_mask = msg.fields.gps_glonass_mask;

  return out;
}

novatel_oem6_msgs::msg::INSPVAS convertINSPVAS(
    const mrpt::obs::gnss::Message_NV_OEM6_INSPVAS& msg, const std_msgs::msg::Header& header)
{
  novatel_oem6_msgs::msg::INSPVAS out;
  out.header = header;

  out.week = msg.fields.week;
  out.seconds_in_week = msg.fields.seconds_in_week;
  out.lat = msg.fields.lat;
  out.lon = msg.fields.lon;
  out.hgt = msg.fields.hgt;
  out.vel_north = msg.fields.vel_north;
  out.vel_east = msg.fields.vel_east;
  out.vel_up = msg.fields.vel_up;
  out.roll = msg.fields.roll;
  out.pitch = msg.fields.pitch;
  out.azimuth = msg.fields.azimuth;
  out.ins_status = msg.fields.ins_status;

  return out;
}

novatel_oem6_msgs::msg::INSCOVS convertINSCOVS(
    const mrpt::obs::gnss::Message_NV_OEM6_INSCOVS& msg, const std_msgs::msg::Header& header)
{
  novatel_oem6_msgs::msg::INSCOVS out;
  out.header = header;

  out.week = msg.fields.week;
  out.seconds_in_week = msg.fields.seconds_in_week;

  for (int i = 0; i < 9; i++)
  {
    out.pos_cov[i] = msg.fields.pos_cov[i];
    out.att_cov[i] = msg.fields.att_cov[i];
    out.vel_cov[i] = msg.fields.vel_cov[i];
  }

  return out;
}

novatel_oem6_msgs::msg::RAWIMUS convertRAWIMUS(
    const mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS& msg, const std_msgs::msg::Header& header)
{
  novatel_oem6_msgs::msg::RAWIMUS out;
  out.header = header;

  out.week = msg.fields.week;
  out.week_seconds = msg.fields.week_seconds;
  out.imu_status = msg.fields.imu_status;
  out.accel_z = msg.fields.accel_z;
  out.accel_y_neg = msg.fields.accel_y_neg;
  out.accel_x = msg.fields.accel_x;
  out.gyro_z = msg.fields.gyro_z;
  out.gyro_y_neg = msg.fields.gyro_y_neg;
  out.gyro_x = msg.fields.gyro_x;

  return out;
}

novatel_oem6_msgs::msg::MARKPOS convertMARKPOS(
    const mrpt::obs::gnss::Message_NV_OEM6_MARKPOS& msg, const std_msgs::msg::Header& header)
{
  novatel_oem6_msgs::msg::MARKPOS out;
  out.header = header;

  out.solution_stat = msg.fields.solution_stat;
  out.position_type = msg.fields.position_type;
  out.lat = msg.fields.lat;
  out.lon = msg.fields.lon;
  out.hgt = msg.fields.hgt;
  out.undulation = msg.fields.undulation;
  out.datum_id = msg.fields.datum_id;
  out.lat_sigma = msg.fields.lat_sigma;
  out.lon_sigma = msg.fields.lon_sigma;
  out.hgt_sigma = msg.fields.hgt_sigma;
  out.base_station_id = std::string(msg.fields.base_station_id, 4);
  out.diff_age = msg.fields.diff_age;
  out.sol_age = msg.fields.sol_age;
  out.num_sats_tracked = msg.fields.num_sats_tracked;
  out.num_sats_sol = msg.fields.num_sats_sol;
  out.num_sats_sol_l1 = msg.fields.num_sats_sol_L1;
  out.num_sats_sol_multi = msg.fields.num_sats_sol_multi;
  out.reserved = msg.fields.reserved;
  out.ext_sol_stat = msg.fields.ext_sol_stat;
  out.galileo_beidou_mask = msg.fields.galileo_beidou_mask;
  out.gps_glonass_mask = msg.fields.gps_glonass_mask;

  return out;
}

novatel_oem6_msgs::msg::MARKTIME convertMARKTIME(
    const mrpt::obs::gnss::Message_NV_OEM6_MARKTIME& msg, const std_msgs::msg::Header& header)
{
  novatel_oem6_msgs::msg::MARKTIME out;
  out.header = header;

  out.week = msg.fields.week;
  out.week_seconds = msg.fields.week_seconds;
  out.clock_offset = msg.fields.clock_offset;
  out.clock_offset_std = msg.fields.clock_offset_std;
  out.utc_offset = msg.fields.utc_offset;
  out.clock_status = msg.fields.clock_status;

  return out;
}

novatel_oem6_msgs::msg::MARK2TIME convertMARK2TIME(
    const mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME& msg, const std_msgs::msg::Header& header)
{
  novatel_oem6_msgs::msg::MARK2TIME out;
  out.header = header;

  out.week = msg.fields.week;
  out.week_seconds = msg.fields.week_seconds;
  out.clock_offset = msg.fields.clock_offset;
  out.clock_offset_std = msg.fields.clock_offset_std;
  out.utc_offset = msg.fields.utc_offset;
  out.clock_status = msg.fields.clock_status;

  return out;
}

novatel_oem6_msgs::msg::IONUTC convertIONUTC(
    const mrpt::obs::gnss::Message_NV_OEM6_IONUTC& msg, const std_msgs::msg::Header& header)
{
  novatel_oem6_msgs::msg::IONUTC out;
  out.header = header;

  out.a0 = msg.fields.a0;
  out.a1 = msg.fields.a1;
  out.a2 = msg.fields.a2;
  out.a3 = msg.fields.a3;
  out.b0 = msg.fields.b0;
  out.b1 = msg.fields.b1;
  out.b2 = msg.fields.b2;
  out.b3 = msg.fields.b3;
  out.utc_wn = msg.fields.utc_wn;
  out.tot = msg.fields.tot;
  out.a0_utc = msg.fields.A0;
  out.a1_utc = msg.fields.A1;
  out.wn_lsf = msg.fields.wn_lsf;
  out.dn = msg.fields.dn;
  out.deltat_ls = msg.fields.deltat_ls;
  out.deltat_lsf = msg.fields.deltat_lsf;
  out.reserved = msg.fields.reserved;

  return out;
}

novatel_oem6_msgs::msg::RAWEPHEM convertRAWEPHEM(
    const mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM& msg, const std_msgs::msg::Header& header)
{
  novatel_oem6_msgs::msg::RAWEPHEM out;
  out.header = header;

  out.sat_prn = msg.fields.sat_prn;
  out.ref_week = msg.fields.ref_week;
  out.ref_secs = msg.fields.ref_secs;

  std::copy(
      std::begin(msg.fields.subframe1), std::end(msg.fields.subframe1), out.subframe1.begin());
  std::copy(
      std::begin(msg.fields.subframe2), std::end(msg.fields.subframe2), out.subframe2.begin());
  std::copy(
      std::begin(msg.fields.subframe3), std::end(msg.fields.subframe3), out.subframe3.begin());

  return out;
}

novatel_oem6_msgs::msg::RXSTATUS convertRXSTATUS(
    const mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS& msg, const std_msgs::msg::Header& header)
{
  novatel_oem6_msgs::msg::RXSTATUS out;
  out.header = header;

  out.error = msg.fields.error;
  out.num_stats = msg.fields.num_stats;
  out.rxstat = msg.fields.rxstat;
  out.rxstat_pri = msg.fields.rxstat_pri;
  out.rxstat_set = msg.fields.rxstat_set;
  out.rxstat_clear = msg.fields.rxstat_clear;
  out.aux1stat = msg.fields.aux1stat;
  out.aux1stat_pri = msg.fields.aux1stat_pri;
  out.aux1stat_set = msg.fields.aux1stat_set;
  out.aux1stat_clear = msg.fields.aux1stat_clear;
  out.aux2stat = msg.fields.aux2stat;
  out.aux2stat_pri = msg.fields.aux2stat_pri;
  out.aux2stat_set = msg.fields.aux2stat_set;
  out.aux2stat_clear = msg.fields.aux2stat_clear;
  out.aux3stat = msg.fields.aux3stat;
  out.aux3stat_pri = msg.fields.aux3stat_pri;
  out.aux3stat_set = msg.fields.aux3stat_set;
  out.aux3stat_clear = msg.fields.aux3stat_clear;

  return out;
}

}  // anonymous namespace

// Updated sensor configuration with improved INS initialization sequence
// Key changes from original:
// 1. Added INSCOMMAND ENABLE to actually start the INS
// 2. Removed static SETINITAZIMUTH from config (sent dynamically based on INS
// state)
// 3. Added elevation cutoff to reduce multipath
// 4. Changed default baud rate to 115200 (typical for USB)
// 5. Added INSATT log for attitude monitoring
// 6. Added RXSTATUS for diagnostics
const char* sensorConfig = R""""(
# Configuration INI file block for mrpt-hwdrivers sensor.
# Read more online:
# https://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/
#

# =======================================================
#  SENSOR: GPS
# =======================================================
[GPS]
driver          = CGPSInterface
process_rate    = ${PROCESS_RATE}  // Hz

sensorLabel     = NOVATEL

# Serial port configuration:
COM_port_LIN = ${NOVATEL_SERIAL_PORT}
baudRate     = ${SERIAL_BAUD_RATE}

# Parser for GNSS data - AUTO will detect NMEA and Novatel binary
parser = AUTO

# Raw data dump (optional, for debugging)
raw_dump_file_prefix = ${RAW_DUMP_FILE}

# Sensor pose (antenna phase center relative to vehicle frame)
pose_x = ${SENSOR_POSE_X}
pose_y = ${SENSOR_POSE_Y}
pose_z = ${SENSOR_POSE_Z}

# Setup commands configuration
setup_cmds_delay = 0.2
setup_cmds_append_CRLF = true

# === INITIALIZATION SEQUENCE ===
# Order matters! Configure before enabling INS.

# 1. Clear all existing logs
setup_cmd1 = UNLOGALL

# 2. PPS output for external sensor synchronization
setup_cmd2 = PPSCONTROL ENABLE POSITIVE 1.0 25000

# 3. NMEA output on COM2 for LiDAR sync (if needed)
setup_cmd3 = serialconfig COM2 9600
setup_cmd4 = LOG COM2 GPRMC ONTIME 1

# 4. Configure NTRIP/RTK correction input port
setup_cmd5 = interfacemode USB3 RTCMV3 none off
setup_cmd6 = LOG USB3 GPGGA ONTIME 1

# 5. Satellite elevation cutoff (degrees) - reduces multipath
setup_cmd7 = ECUTOFF 10

# 6. Enable SBAS corrections
setup_cmd8 = sbascontrol enable auto

# === IMU/INS LEVER ARM CONFIGURATION ===
# CRITICAL: These must be measured accurately for good INS performance

# 7. IMU orientation (defines how IMU is mounted)
# See Novatel manual Table 9 for orientation codes:
#   1: Z up, Y forward    5: Z up, Z forward
#   2: Z up, X forward    6: Z down, Y forward (common vehicle setup)
#   3: Z down, Y forward  7: Z down, X forward
#   4: Z down, X forward  etc.
setup_cmd9 = SETIMUORIENTATION ${SETIMUORIENTATION}

# 8. Vehicle body rotation (rotation from IMU frame to vehicle frame)
# Format: roll pitch azimuth roll_std pitch_std azimuth_std (degrees)
setup_cmd10 = VEHICLEBODYROTATION ${VEHICLEBODYROTATION}
setup_cmd11 = APPLYVEHICLEBODYROTATION enable

# 9. IMU to antenna offset (in IMU frame)
# Format: x y z x_std y_std z_std (meters)
# CRITICAL for RTK+INS fusion accuracy
setup_cmd12 = SETIMUTOANTOFFSET ${SETIMUTOANTOFFSET}

# 10. INS solution output offset from IMU center
# Format: x y z (meters, in vehicle frame)
setup_cmd13 = SETINSOFFSET ${SETINSOFFSET}

# === INS ALIGNMENT CONFIGURATION ===

# 11. Alignment mode - determines how heading is initialized
# Options:
#   UNAIDED     - Stationary alignment, requires SETINITAZIMUTH command
#   KINEMATIC   - Requires motion to determine heading (needs ~20km/h)
#   AUTOMATIC   - Receiver decides based on available aiding
#   AIDED_TRANSFER - Use heading from external source
#   STATIC      - High-accuracy stationary alignment (longer time)
setup_cmd14 = ALIGNMENTMODE ${ALIGNMENT_MODE}

# 12. Enable the INS engine - REQUIRED to start INS processing!
setup_cmd15 = INSCOMMAND ENABLE

# === OUTPUT LOG CONFIGURATION ===

# 13. Position logs (high rate for navigation)
setup_cmd16 = LOG USB1 BESTPOSB ONTIME 0.05
setup_cmd17 = LOG USB1 BESTVELB ONTIME 0.1

# 14. INS solution (position + velocity + attitude)
setup_cmd18 = LOG USB1 INSPVASB ONTIME 0.05
setup_cmd19 = LOG USB1 INSATTB ONTIME 0.1

# 15. Raw measurements for post-processing and RTK
setup_cmd20 = LOG USB1 RANGECMPB ONTIME 1
setup_cmd21 = LOG USB1 RAWEPHEMB ONCHANGED
setup_cmd22 = LOG USB1 IONUTCB ONCHANGED

# 16. Diagnostics and status
setup_cmd23 = LOG USB1 VERSION ONCE
setup_cmd24 = LOG USB1 RXSTATUSB ONTIME 10

# 17. GGA for NTRIP feedback
setup_cmd25 = LOG USB1 GPGGARTK ONTIME 1

# Shutdown
shutdown_cmd1 = UNLOGALL


# =======================================================
#  VIRTUAL SENSOR: NTRIP CLIENT
# =======================================================
[NTRIP]
driver       = CNTRIPEmitter
process_rate = ${PROCESS_RATE}

# Serial port for NTRIP data output to Novatel
COM_port_LIN = ${NOVATEL_NTRIP_INPUT_SERIAL_PORT}
baudRate     = 9600

# NTRIP caster configuration
server     = ${NTRIP_SERVER}
port       = ${NTRIP_PORT}
mountpoint = ${NTRIP_MOUNT_POINT}
user       = ${NTRIP_USER}
password   = ${NTRIP_PASSWORD}

)"""";

int main(int argc, char** argv)
{
  try
  {
    // Init ROS:
    rclcpp::init(argc, argv);

    auto node = std::make_shared<mrpt_sensors::GenericSensorNode>(node_name);

    // Node state for INS management
    NovatelNodeState state;
    auto logger = node->get_logger();

    // Declare additional parameters for external IMU azimuth support
    node->declare_parameter("external_imu_topic", "");
    node->declare_parameter("use_external_imu_azimuth", false);
    node->declare_parameter("external_imu_max_age_sec", 2.0);
    node->declare_parameter("external_imu_azimuth_deg_for_yaw_zero", 0.0);
    node->declare_parameter("alignment_mode", "UNAIDED");
    node->declare_parameter("novatel_init_azimuth", "0.0 25.0");  // "azimuth_deg std_deg"

    // Get parameters
    std::string external_imu_topic = node->get_parameter("external_imu_topic").as_string();
    state.use_external_imu_azimuth = node->get_parameter("use_external_imu_azimuth").as_bool();
    state.external_imu_max_age_sec = node->get_parameter("external_imu_max_age_sec").as_double();
    state.external_imu_azimuth_deg_for_yaw_zero =
        node->get_parameter("external_imu_azimuth_deg_for_yaw_zero").as_double();

    // Parse manual azimuth parameter
    std::string az_str = node->get_parameter("novatel_init_azimuth").as_string();
    if (sscanf(
            az_str.c_str(), "%lf %lf", &state.manual_init_azimuth_deg,
            &state.manual_init_azimuth_std_deg) < 1)
    {
      state.manual_init_azimuth_deg = 0.0;
      state.manual_init_azimuth_std_deg = 25.0;
    }

    RCLCPP_INFO(
        logger, "Manual azimuth: %.2f deg (std: %.2f deg)", state.manual_init_azimuth_deg,
        state.manual_init_azimuth_std_deg);

    // Subscribe to external IMU if topic is specified
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    if (!external_imu_topic.empty())
    {
      RCLCPP_INFO(logger, "Subscribing to external IMU topic: %s", external_imu_topic.c_str());

      imu_sub = node->create_subscription<sensor_msgs::msg::Imu>(
          external_imu_topic, rclcpp::SensorDataQoS(),
          [&state, &logger](const sensor_msgs::msg::Imu::SharedPtr msg)
          {
            std::lock_guard<std::mutex> lock(state.mutex);

            // Check if orientation is valid
            if (msg->orientation_covariance[0] < 0)
            {
              state.external_imu.valid = false;
              return;
            }

            state.external_imu.azimuth_deg =
                quaternionToAzimuthDeg(*msg, state.external_imu_azimuth_deg_for_yaw_zero);

            // Extract yaw uncertainty from covariance
            if (msg->orientation_covariance[8] > 0)
            {
              state.external_imu.azimuth_std_deg =
                  std::sqrt(msg->orientation_covariance[8]) * 180.0 / M_PI;
            }
            else
            {
              state.external_imu.azimuth_std_deg = 10.0;
            }

            state.external_imu.timestamp = msg->header.stamp;
            state.external_imu.valid = true;

            RCLCPP_DEBUG(
                logger, "External IMU azimuth: %.2f deg (std: %.2f deg)",
                state.external_imu.azimuth_deg, state.external_imu.azimuth_std_deg);
          });
    }

    // Service to send custom commands to Novatel
    auto send_command_srv = node->create_service<novatel_oem6_msgs::srv::SendNovatelCommand>(
        "~/send_novatel_command",
        [&state, &logger](
            const std::shared_ptr<novatel_oem6_msgs::srv::SendNovatelCommand::Request> request,
            std::shared_ptr<novatel_oem6_msgs::srv::SendNovatelCommand::Response> response)
        {
          if (!state.gps_interface)
          {
            RCLCPP_ERROR(logger, "GPS interface not available for sending commands");
            response->done_ok = false;
            return;
          }

          std::string cmd = request->command + "\r\n";
          RCLCPP_INFO(logger, "Sending custom command: %s", request->command.c_str());

          response->done_ok = state.gps_interface->sendCustomCommand(cmd.c_str(), cmd.length());

          if (response->done_ok)
          {
            RCLCPP_INFO(logger, "Command sent successfully");
          }
          else
          {
            RCLCPP_ERROR(logger, "Failed to send command");
          }
        });

    RCLCPP_INFO(logger, "Service '~/send_novatel_command' ready");

    // Publishers for Novatel binary messages
    NovatelPublishers nvt_pubs;

    // Custom GPS observation processing

    node->custom_process_sensor =
        [&node, &state, &logger, &nvt_pubs](const mrpt::obs::CObservation::Ptr& obs)
    {
      auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(obs);
      if (!o)
      {
        return;
      }

      std_msgs::msg::Header msgHeader;
      msgHeader.frame_id = node->sensor_frame_id();
      msgHeader.stamp = mrpt::ros2bridge::toROS(obs->timestamp);

      node->ensure_publisher_exists<sensor_msgs::msg::NavSatFix>(node->gps_publisher_);

      // Publish Novatel binary messages as ROS2 messages
      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_BESTPOS>())
      {
        const auto& msg = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_BESTPOS>();
        ensurePublisher<novatel_oem6_msgs::msg::BESTPOS>(
            nvt_pubs.bestpos, node.get(), "~/novatel/bestpos");
        nvt_pubs.bestpos->publish(convertBESTPOS(msg, msgHeader));
      }

      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_INSPVAS>())
      {
        const auto& msg = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_INSPVAS>();
        ensurePublisher<novatel_oem6_msgs::msg::INSPVAS>(
            nvt_pubs.inspvas, node.get(), "~/novatel/inspvas");
        nvt_pubs.inspvas->publish(convertINSPVAS(msg, msgHeader));

        auto ins_status = static_cast<InsStatus>(msg.fields.ins_status);
        {
          std::lock_guard<std::mutex> lock(state.mutex);
          if (ins_status != state.last_ins_status)
          {
            RCLCPP_INFO(
                logger, "INS status: %s -> %s", insStatusToString(state.last_ins_status),
                insStatusToString(ins_status));
            state.last_ins_status = ins_status;

            if (ins_status == InsStatus::INS_WAITING_AZIMUTH ||
                ins_status == InsStatus::INS_ALIGNING || ins_status == InsStatus::INS_INACTIVE)
            {
              state.azimuth_sent = false;
            }
          }
        }
        checkAndSendAzimuth(state, *node, logger);
      }

      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_INSCOVS>())
      {
        const auto& msg = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_INSCOVS>();
        ensurePublisher<novatel_oem6_msgs::msg::INSCOVS>(
            nvt_pubs.inscovs, node.get(), "~/novatel/inscovs");
        nvt_pubs.inscovs->publish(convertINSCOVS(msg, msgHeader));
      }

      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS>())
      {
        const auto& msg = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_RAWIMUS>();
        ensurePublisher<novatel_oem6_msgs::msg::RAWIMUS>(
            nvt_pubs.rawimus, node.get(), "~/novatel/rawimus");
        nvt_pubs.rawimus->publish(convertRAWIMUS(msg, msgHeader));
      }

      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_MARKPOS>())
      {
        const auto& msg = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_MARKPOS>();
        ensurePublisher<novatel_oem6_msgs::msg::MARKPOS>(
            nvt_pubs.markpos, node.get(), "~/novatel/markpos");
        nvt_pubs.markpos->publish(convertMARKPOS(msg, msgHeader));
      }

      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_MARKTIME>())
      {
        const auto& msg = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_MARKTIME>();
        ensurePublisher<novatel_oem6_msgs::msg::MARKTIME>(
            nvt_pubs.marktime, node.get(), "~/novatel/marktime");
        nvt_pubs.marktime->publish(convertMARKTIME(msg, msgHeader));
      }

      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME>())
      {
        const auto& msg = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_MARK2TIME>();
        ensurePublisher<novatel_oem6_msgs::msg::MARK2TIME>(
            nvt_pubs.mark2time, node.get(), "~/novatel/mark2time");
        nvt_pubs.mark2time->publish(convertMARK2TIME(msg, msgHeader));
      }

      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_IONUTC>())
      {
        const auto& msg = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_IONUTC>();
        ensurePublisher<novatel_oem6_msgs::msg::IONUTC>(
            nvt_pubs.ionutc, node.get(), "~/novatel/ionutc");
        nvt_pubs.ionutc->publish(convertIONUTC(msg, msgHeader));
      }

      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM>())
      {
        const auto& msg = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_RAWEPHEM>();
        ensurePublisher<novatel_oem6_msgs::msg::RAWEPHEM>(
            nvt_pubs.rawephem, node.get(), "~/novatel/rawephem");
        nvt_pubs.rawephem->publish(convertRAWEPHEM(msg, msgHeader));
      }

      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS>())
      {
        const auto& msg = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_RXSTATUS>();
        ensurePublisher<novatel_oem6_msgs::msg::RXSTATUS>(
            nvt_pubs.rxstatus, node.get(), "~/novatel/rxstatus");
        nvt_pubs.rxstatus->publish(convertRXSTATUS(msg, msgHeader));
      }

      std::optional<std::array<double, 3>> enu_sigmas;
      if (o->hasMsgClass<mrpt::obs::gnss::Message_NV_OEM6_BESTPOS>())
      {
        const auto& bestpos = o->getMsgByClass<mrpt::obs::gnss::Message_NV_OEM6_BESTPOS>();

        static uint32_t last_sol_stat = 0xFFFFFFFF;
        static uint32_t last_pos_type = 0xFFFFFFFF;

        if (bestpos.fields.solution_stat != last_sol_stat ||
            bestpos.fields.position_type != last_pos_type)
        {
          last_sol_stat = bestpos.fields.solution_stat;
          last_pos_type = bestpos.fields.position_type;
          RCLCPP_INFO(logger, "BESTPOS: sol_status=%u, pos_type=%u", last_sol_stat, last_pos_type);
        }
        auto& enu_sgm = enu_sigmas.emplace();
        enu_sgm[0] = bestpos.fields.lon_sigma;
        enu_sgm[1] = bestpos.fields.lat_sigma;
        enu_sgm[2] = bestpos.fields.hgt_sigma;
      }

      if (VERBOSE)
      {
        constexpr double DEBUG_LOG_THROTTLE_PERIOD = 5.0;
        static double debugLogThrottleLastStamp = 0;
        if (const double tNow = mrpt::Clock::nowDouble();
            tNow > debugLogThrottleLastStamp + DEBUG_LOG_THROTTLE_PERIOD)
        {
          debugLogThrottleLastStamp = tNow;
          std::stringstream ss;
          o->getDescriptionAsText(ss);
          RCLCPP_INFO(node->get_logger(), "%s", ss.str().c_str());
        }
      }

      auto header = node->create_header(*o);
      auto msg = sensor_msgs::msg::NavSatFix();

      bool valid = mrpt::ros2bridge::toROS(*o, header, msg);
      if (valid)
      {
        if (enu_sigmas.has_value())
        {
          msg.position_covariance_type =
              sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

          msg.position_covariance.fill(0);
          msg.position_covariance[0 * 3 + 0] = mrpt::square((*enu_sigmas)[0]);
          msg.position_covariance[1 * 3 + 1] = mrpt::square((*enu_sigmas)[1]);
          msg.position_covariance[2 * 3 + 2] = mrpt::square((*enu_sigmas)[2]);
        }

        node->gps_publisher_->publish(msg);
      }
    };

    // Initialize node with sensor configuration
    node->init(
        sensorConfig,
        {
            {"process_rate", "PROCESS_RATE", "100", false},
            {"novatel_main_serial_port", "NOVATEL_SERIAL_PORT", "", true},
            {"novatel_ntrip_serial_port", "NOVATEL_NTRIP_INPUT_SERIAL_PORT", "", true},
            {"serial_baud_rate", "SERIAL_BAUD_RATE", "115200", false},
            {"raw_dump_file", "RAW_DUMP_FILE", "", false},
            {"novatel_imu_orientation", "SETIMUORIENTATION", "6", false},
            {"novatel_veh_body_rotation", "VEHICLEBODYROTATION", "0.0 0.0 0.0 0.5 0.5 0.5", false},
            {"novatel_imu_to_ant_offset", "SETIMUTOANTOFFSET", "0.0 0.0 0.0 0.05 0.05 0.05", false},
            {"novatel_ins_offset", "SETINSOFFSET", "0.0 0.0 0.0", false},
            {"alignment_mode", "ALIGNMENT_MODE", "UNAIDED", false},
            {"sensor_pose_x", "SENSOR_POSE_X", "0", false},
            {"sensor_pose_y", "SENSOR_POSE_Y", "0", false},
            {"sensor_pose_z", "SENSOR_POSE_Z", "0", false},

            {"ntrip_server", "NTRIP_SERVER", "www.euref-ip.net", false},
            {"ntrip_port", "NTRIP_PORT", "2101", false},
            {"ntrip_mount_point", "NTRIP_MOUNT_POINT", "ALME00ESP0", false},
            {"ntrip_user", "NTRIP_USER", "", false},
            {"ntrip_password", "NTRIP_PASSWORD", "", false},
        },
        {"GPS", "NTRIP"});

    // Get pointer to GPS interface for sending commands
    auto* gps = node->getSensor<mrpt::hwdrivers::CGPSInterface>(0);
    if (gps)
    {
      state.gps_interface = gps;
      RCLCPP_INFO(logger, "GPS interface acquired for runtime commands");
    }
    else
    {
      RCLCPP_WARN(logger, "Could not get GPS interface - runtime commands disabled");
    }

    node->run();

    rclcpp::shutdown();
    return 0;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger(""), "Exception in " << node_name << " main(): " << e.what());
    return 1;
  }
}