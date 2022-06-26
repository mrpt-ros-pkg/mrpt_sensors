/* mrpt_sensors ROS package
 *
 * Copyright 2021-2022, Jose Luis Blanco Claraco
 * License: BSD 3-Clause License
 */

#include <ros/ros.h>
#include <ros/console.h>
#include "mrpt_sensorlib/mrpt_sensorlib.h"

int main(int argc, char** argv)
{
	try
	{
		// Init ROS:
		ros::init(argc, argv, "mrpt_generic_sensor");

		ROS_INFO("About to init GenericSensorNode...");
		mrpt_sensorlib::GenericSensorNode node;

		ROS_INFO("Calling GenericSensorNode::init()");
		node.init_from_config_file(argc, argv);
		ROS_INFO("Calling GenericSensorNode::run()");
		node.run();

		return 0;
	}
	catch (std::exception& e)
	{
		ROS_ERROR_STREAM(
			"Exception in mrpt_generic_sensor main(): " << e.what());
		return 1;
	}
}
