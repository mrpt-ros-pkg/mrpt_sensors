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
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt_sensors
{
class GenericSensorNode : public rclcpp::Node
{
   public:
	GenericSensorNode();
	~GenericSensorNode();

	void init();
	void run();

   private:
	std::string cfgfilename_{"sensor.ini"}, cfg_section_{"SENSOR1"};
	mrpt::hwdrivers::CGenericSensor::Ptr sensor_;
	std::string out_rawlog_prefix_;

	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
	rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;

	virtual void process_sensor_specific()
	{ /*do nothing by default*/
	}

	virtual void init_sensor_specific()
	{ /*do nothing by default*/
	}

	void process_observation(const mrpt::obs::CObservation::Ptr& o);

	void process(const mrpt::obs::CObservationGPS& o);
};
}  // namespace mrpt_sensors
