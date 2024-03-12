/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <rclcpp/rclcpp.hpp>
#include "mrpt_sensorlib/mrpt_sensorlib.h"

const char* node_name = "mrpt_sensor_gnns_nmea";
const char* sensorConfig = R""""(
# Configuration INI file block for mrpt-hwdrivers sensor.
# Read more online:
# https://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/
#
[SENSOR]
driver			= CGPSInterface
process_rate	= ${PROCESS_RATE} // Hz

# Serial port configuration:
COM_port_WIN = ${SERIAL_PORT}
COM_port_LIN = ${SERIAL_PORT}
baudRate     = ${SERIAL_BAUD_RATE}   // The baudrate of the communications (typ. 4800 or 9600 bauds)

# (Default:true) Whether to append the GNNS message type to CObservation `sensorLabel` field
sensor_label_append_msg_type = true

# Select a parser for GNSS data:
# Up-to-date list of supported parsers available in https://reference.mrpt.org/devel/classmrpt_1_1hwdrivers_1_1_c_g_p_s_interface.html
parser =  AUTO

# If uncommented and non-empty, raw binary/ascii data received from the serial port will be also dumped 
# into a file named after this prefix, plus date/time and extension `.gps`.
#raw_dump_file_prefix = RAWGPS

# 3D position (and orientation, for GNSS+IMUs) of the sensed point (antenna phase center) relative to the vehicle/robot frame:
pose_x       = ${SENSOR_POSE_X}      // (meters)
pose_y       = ${SENSOR_POSE_Y}
pose_z       = ${SENSOR_POSE_Z}

# Optional: list of initial commands to be sent to the GNSS receiver to set it up.
# An arbitrary number of commands can be defined, but their names must be "setup_cmd%d" starting at "1". 
# Commands will be sent by index order. Binary commands instead of ASCII strings can be set programmatically, not from a config file.
# custom_cmds_delay   = 0.1   // (Default=0.1) Delay in seconds between consecutive set-up commands
# custom_cmds_append_CRLF = true    // (Default:true) Append "\r\n" to each command
# setup_cmd1 = XXXXX
# setup_cmd2 = XXXXX
# setup_cmd3 = XXXXX
# Optional: list of commands to be sent upon disconnection (e.g. object destructor)
# shutdown_cmd1 = XXXX
# shutdown_cmd2 = XXXX

)"""";

int main(int argc, char** argv)
{
	try
	{
		// Init ROS:
		rclcpp::init(argc, argv);

		auto node =
			std::make_shared<mrpt_sensors::GenericSensorNode>(node_name);

		node->init(
			sensorConfig,
			{
				{"process_rate", "PROCESS_RATE", "500", false},
				{"serial_port", "SERIAL_PORT", "", true},
				{"serial_baud_rate", "SERIAL_BAUD_RATE", "4800", false},
				{"sensor_pose_x", "SENSOR_POSE_X", "0", false},
				{"sensor_pose_y", "SENSOR_POSE_Y", "0", false},
				{"sensor_pose_z", "SENSOR_POSE_Z", "0", false},
			},
			"SENSOR");

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
