/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <rclcpp/rclcpp.hpp>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/core/bits_math.h>  // square()
#include "mrpt_sensorlib/mrpt_sensorlib.h"

const char* node_name = "mrpt_sensor_bumblebee_stereo";
const char* sensorConfig = R""""(
# Configuration INI file block for mrpt-hwdrivers sensor.
# Read more online:
# https://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/
#
[SENSOR]
driver			= CCameraSensor
process_rate	= ${PROCESS_RATE} // Hz

sensorLabel		= ${SENSOR_LABEL}

grabber_type	= bumblebee_dc1394

preview_decimation = ${CAMERA_PREVIEW_DECIMATION}

pose_x       = ${SENSOR_POSE_X}      // (meters)
pose_y       = ${SENSOR_POSE_Y}
pose_z       = ${SENSOR_POSE_Z}
pose_yaw     = ${SENSOR_POSE_YAW}      // (deg)
pose_pitch   = ${SENSOR_POSE_PITCH}
pose_z       = ${SENSOR_POSE_ROLL}

bumblebee_dc1394_camera_guid   = ${BUMBLEBEE_DC1394_CAMERA_GUID}  // 0 (or not present): the first camera. A hexadecimal number (0x11223344): The GUID of the camera to open
bumblebee_dc1394_camera_unit   = 0  // 0 (or not present): the first camera. 0,1,2,...: The unit number (within the given GUID) of the camera to open (Stereo cameras: 0 or 1)
bumblebee_dc1394_framerate     = ${BUMBLEBEE_DC1394_FRAMERATE} // eg: 7.5, 15, 30, 60, etc... For possibilities see mrpt::hwdrivers::TCaptureOptions_dc1394

capture_grayscale  	= false

# For externaly stored images, the format of image files (default=jpg)
external_images_format = png

# For externaly stored images: whether to spawn an independent process to save the image files.
external_images_own_thread  = true

# If external_images_own_thread=1, this changes the number of threads to launch
#  to save image files. The default is determined from mrpt::system::getNumberOfProcessors()
#  and should be OK unless you want to save processor time for other things.
#external_images_own_thread_count = 2    // >=1

)"""";

namespace
{
// We will emit two ROS messages per stereo observation.
void process_gps(
	mrpt_sensors::GenericSensorNode& node,
	const mrpt::obs::CObservation::Ptr& obs)
{
	using namespace std::string_literals;

	auto o =
		std::dynamic_pointer_cast<mrpt::obs::CObservationStereoImages>(obs);
	ASSERT_(o);

	auto header = node.create_header(*o);
	header.frame_id += "_left"s;
	sensor_msgs::msg::Image msgLeft =
		mrpt::ros2bridge::toROS(o->imageLeft, header);

	header = node.create_header(*o);
	header.frame_id += "_right"s;
	sensor_msgs::msg::Image msgRight =
		mrpt::ros2bridge::toROS(o->imageRight, header);

	// publish them:
	node.ensure_publisher_exists<sensor_msgs::msg::Image>(
		node.images_publisher_["left"], "_left");

	node.images_publisher_["left"]->publish(msgLeft);

	if (o->hasImageRight)
	{
		node.ensure_publisher_exists<sensor_msgs::msg::Image>(
			node.images_publisher_["right"], "_right");
		node.images_publisher_["right"]->publish(msgRight);
	}
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
			[&node](const mrpt::obs::CObservation::Ptr& o) {
				process_gps(*node, o);
			};

		node->init(
			sensorConfig,
			{
				{"process_rate", "PROCESS_RATE", "80", false},
				{"sensor_label", "SENSOR_LABEL", "stereo", false},
				{"dc1394_framerate", "BUMBLEBEE_DC1394_FRAMERATE", "15", false},

				{"dc1394_camera_guid", "BUMBLEBEE_DC1394_CAMERA_GUID", "0",
				 false},
				// 0 (or not present): the first camera. A hexadecimal
				// number (0x11223344): The GUID of the camera to open

				{"camera_preview_decimation", "CAMERA_PREVIEW_DECIMATION", "0",
				 false},

				{"sensor_pose_x", "SENSOR_POSE_X", "0", false},
				{"sensor_pose_y", "SENSOR_POSE_Y", "0", false},
				{"sensor_pose_z", "SENSOR_POSE_Z", "0", false},
				{"sensor_pose_yaw", "SENSOR_POSE_YAW", "0", false},
				{"sensor_pose_pitch", "SENSOR_POSE_PITCH", "0", false},
				{"sensor_pose_roll", "SENSOR_POSE_ROLL", "0", false},
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
