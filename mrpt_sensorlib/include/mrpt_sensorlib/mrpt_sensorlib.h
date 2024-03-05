/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <mrpt/hwdrivers/CGenericSensor.h>

namespace mrpt_sensors
{
class GenericSensorNode : public rclcpp::Node
{
   private:
	std::string cfgfilename_{"sensor.ini"}, cfg_section_{"SENSOR1"};
	mrpt::hwdrivers::CGenericSensor::Ptr sensor_;
	std::string out_rawlog_prefix_;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

	virtual void process_sensor_specific()
	{ /*do nothing by default*/
	}

	virtual void init_sensor_specific()
	{ /*do nothing by default*/
	}

   public:
	GenericSensorNode();

	~GenericSensorNode();

	void init();

	void run();
};
}  // namespace mrpt_sensors
