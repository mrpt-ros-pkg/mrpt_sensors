
#include <ros/ros.h>
#include <ros/console.h>
#include "mrpt_sensorlib/mrpt_sensorlib.h"

int main(int argc, char** argv)
{
	try
	{
		mrpt_sensors::GenericSensorNode node;

		node.init(argc, argv);
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
