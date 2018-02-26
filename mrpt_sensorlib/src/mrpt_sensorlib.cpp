
#include "mrpt_sensorlib/mrpt_sensorlib.h"

using namespace mrpt_sensors;

GenericSensorNode::GenericSensorNode() {}

GenericSensorNode::~GenericSensorNode() {}

void GenericSensorNode::init(int argc, char** argv)
{
	// Init ROS:
	ros::init(argc, argv, nodename_);

	// Load parameters:

	// Call sensor factory:

	// Initialize sensor:

	// Custom init:
	this->init_sensor_specific();
}

/** Infinite loop for main(). */
void GenericSensorNode::run()
{
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
	}
}
