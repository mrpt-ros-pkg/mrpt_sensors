/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2024, Individual contributors, see commit authors   |
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
   +------------------------------------------------------------------------+ */

#include <mrpt/core/bits_math.h>  // square()
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/time.h>

#include <rclcpp/rclcpp.hpp>

#include "mrpt_sensorlib/mrpt_sensorlib.h"

const char* node_name = "mrpt_sensor_gnss_novatel";
const char* sensorConfig = R""""(
# Configuration INI file block for mrpt-hwdrivers sensor.
# Read more online:
# https://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/
#

# =======================================================
#  SENSOR: GPS
# =======================================================
[GPS]
#rawlog-grabber-ignore = true
driver			= CGPSInterface
process_rate	= ${PROCESS_RATE}  // Hz

sensorLabel		= NOVATEL

# Serial port configuration:
#COM_port_WIN = COM11
COM_port_LIN = ${NOVATEL_SERIAL_PORT}
baudRate     = ${SERIAL_BAUD_RATE}   // The baudrate of the communications (typ. 4800 or 9600 bauds)

# Select a parser for GNSS data:
# Up-to-date list of supported parsers available in http://reference.mrpt.org/devel/classmrpt_1_1hwdrivers_1_1_c_g_p_s_interface.html
parser =  AUTO

# If uncommented and non-empty, raw binary/ascii data received from the serial port will be also dumped
# into a file named after this prefix, plus date/time and extension `.gps`.
raw_dump_file_prefix = ${RAW_DUMP_FILE}

# 3D position (and orientation, for GNSS+IMUs) of the sensed point (antenna phase center) relative to the vehicle/robot frame:
pose_x       = ${SENSOR_POSE_X}      // (meters)
pose_y       = ${SENSOR_POSE_Y}
pose_z       = ${SENSOR_POSE_Z}

# Optional: list of initial commands to be sent to the GNSS receiver to set it up.
# An arbitrary number of commands can be defined, but their names must be "setup_cmd%d" starting at "1".
# Commands will be sent by index order. Binary commands instead of ASCII strings can be set programatically, not from a config file.
setup_cmds_delay   = 0.2   // (Default=0.1) Delay in seconds between consecutive set-up commands
setup_cmds_append_CRLF = true    // (Default:true) Append "\r\n" to each command
setup_cmd1 = UNLOGALL

# Generate PPS, for wired LIDAR (if present)
setup_cmd2 = PPSCONTROL ENABLE POSITIVE 1.0 25000

# Generate RMC frames for the LiDAR
setup_cmd3 = serialconfig COM2 9600
setup_cmd4 = LOG COM2 GPRMC ONTIME 1

# Config RTK NTRIP inputs:
# INTERFACEMODE [port] rxtype txtype [responses]
setup_cmd5 = interfacemode USB3 RTCMV3 none off
setup_cmd6 = LOG USB3 GPGGA ONTIME 1

# Set IMU Orientation, Z points down
setup_cmd7= SETIMUORIENTATION ${SETIMUORIENTATION}

# Set Vehicle to Body Rotation
setup_cmd8= VEHICLEBODYROTATION ${VEHICLEBODYROTATION}
setup_cmd9=	APPLYVEHICLEBODYROTATION enable

# Set Lever Arm Offset
setup_cmd10= SETIMUTOANTOFFSET ${SETIMUTOANTOFFSET}
setup_cmd11= SETINSOFFSET ${SETINSOFFSET}

# Stationary Alignment
setup_cmd12= ALIGNMENTMODE UNAIDED
setup_cmd13= SETINITAZIMUTH ${SETINITAZIMUTH}

# Logs to save:
setup_cmd14 = LOG USB1 BESTPOSB ONTIME 0.05
setup_cmd15 = LOG USB1 INSPVASB ONTIME 0.05
setup_cmd16 = LOG USB1 RANGECMPB ONTIME 1
setup_cmd17 = LOG USB1 RAWEPHEMB ONCHANGED
setup_cmd18 = LOG USB1 IONUTCB ONCHANGED
setup_cmd19 = LOG USB1 VERSION ONCE
setup_cmd20 = LOG USB1 GPGGARTK ONTIME 1

# Use SBAS
setup_cmd21 = sbascontrol enable auto

#end
shutdown_cmd1 = UNLOGALL


# =======================================================
#  VIRTUAL SENSOR: NTRIP
# =======================================================
[NTRIP]
#rawlog-grabber-ignore = true
driver			= CNTRIPEmitter
process_rate	= ${PROCESS_RATE}	// Hz

#COM_port_WIN = COM9         // Serial port where the NTRIP stream will be dumped to.
COM_port_LIN = ${NOVATEL_NTRIP_INPUT_SERIAL_PORT}
baudRate     = 9600

server   = ${NTRIP_SERVER}
port     = ${NTRIP_PORT}
mountpoint = ${NTRIP_MOUNT_POINT}
user = ${NTRIP_USER}
password = ${NTRIP_PASSWORD}

)"""";

namespace
{
// We will emit one ROS message per GGA NMEA frame.
// toROS() below will return false if there is NO GGA frame.
// But we want to keep an eye on other NMEA frames to learn about
// fix status and accuracy, etc.
void process_gps(
    mrpt_sensors::GenericSensorNode& node,
    const mrpt::obs::CObservation::Ptr& obs)
{
    auto o = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(obs);
    ASSERT_(o);

    std_msgs::msg::Header msgHeader;
    msgHeader.frame_id = node.sensor_frame_id();
    msgHeader.stamp = mrpt::ros2bridge::toROS(obs->timestamp);

    node.ensure_publisher_exists<sensor_msgs::msg::NavSatFix>(
        node.gps_publisher_);

#if 1
    std::stringstream ss;
    o->getDescriptionAsText(ss);
    RCLCPP_INFO_STREAM(node.get_logger(), ss.str());
#endif

#if 0
    auto header = node.create_header(*o);

    auto msg = sensor_msgs::msg::NavSatFix();

    bool valid = mrpt::ros2bridge::toROS(*o, header, msg);
    if (!valid) return;

    msg.position_covariance_type =
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    const double sigma_E = HDOP * UERE;
    const double sigma_N = HDOP * UERE;
    const double sigma_U = VDOP * UERE;

    RCLCPP_DEBUG_THROTTLE(
        node.get_logger(), *node.get_clock(), 5000,
        "Uncertainties: sigma_xy=%.03f m sigma_z=%.03f m", sigma_E, sigma_U);

    msg.position_covariance.fill(0.0);
    msg.position_covariance[0] = mrpt::square(sigma_E);
    msg.position_covariance[4] = mrpt::square(sigma_N);
    msg.position_covariance[8] = mrpt::square(sigma_U);

    node.gps_publisher_->publish(msg);
#endif
}
}  // namespace

int main(int argc, char** argv)
{
    try
    {
        // Init ROS:
        rclcpp::init(argc, argv);

        auto node =
            std::make_shared<mrpt_sensors::GenericSensorNode>(node_name);

        node->custom_process_sensor =
            [&node](const mrpt::obs::CObservation::Ptr& o)
        { process_gps(*node, o); };

        node->init(
            sensorConfig,
            {
                {"process_rate", "PROCESS_RATE", "500", false},
                {"novatel_main_serial_port", "NOVATEL_SERIAL_PORT", "", true},
                {"novatel_ntrip_serial_port", "NOVATEL_NTRIP_INPUT_SERIAL_PORT",
                 "", true},
                {"serial_baud_rate", "SERIAL_BAUD_RATE", "4800", false},
                {"raw_dump_file", "RAW_DUMP_FILE", "", false},
                {"novatel_imu_orientation", "SETIMUORIENTATION", "6", false},
                {"novatel_veh_body_rotation", "VEHICLEBODYROTATION",
                 "0.000000 0.000000 90.000000 0.000000 0.000000 0.000000",
                 false},
                {"novatel_imu_to_ant_offset", "SETIMUTOANTOFFSET",
                 "-0.28 -0.08 -0.01 0.000000 0.000000 0.000000", false},
                {"novatel_ins_offset", "SETINSOFFSET",
                 "0.000000 0.000000 0.000000", false},
                {"novatel_init_azimuth", "SETINITAZIMUTH", "0.000000 25.000000",
                 false},
                {"sensor_pose_x", "SENSOR_POSE_X", "0", false},
                {"sensor_pose_y", "SENSOR_POSE_Y", "0", false},
                {"sensor_pose_z", "SENSOR_POSE_Z", "0", false},

                {"ntrip_server", "NTRIP_SERVER", "www.euref-ip.net", false},
                {"ntrip_port", "NTRIP_PORT", "\"2101\"", false},
                {"ntrip_mount_point", "NTRIP_MOUNT_POINT", "ALME00ESP0", false},
                {"ntrip_user", "NTRIP_USER", "user", false},
                {"ntrip_password", "NTRIP_PASSWORD", "pass", false},
            },
            {"GPS", "NTRIP"});

        node->run();

        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(""),
            "Exception in " << node_name << " main(): " << e.what());
        return 1;
    }
}
