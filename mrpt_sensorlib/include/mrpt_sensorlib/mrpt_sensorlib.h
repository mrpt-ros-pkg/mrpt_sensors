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
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/obs/obs_frwds.h>
#include <mrpt_msgs/msg/generic_observation.hpp>

namespace mrpt_sensors
{
class GenericSensorNode : public rclcpp::Node
{
   public:
	GenericSensorNode(const std::string& nodeName = "generic_sensor_node");
	~GenericSensorNode();

	/// Initialize from INI file and section given by ROS2 parameters:
	void init();

	struct TemplateParameter
	{
		std::string ros_param_name;
		std::string template_variable;
		std::string default_value;
		bool required;
	};

	/// Initialize from the given template text with a set of ROS2 parameters:
	void init(
		const char* templateText,
		const std::vector<TemplateParameter>& rosParams,
		const std::string& section = "SENSOR");

	/// Initialize from the given configuration source:
	void init(
		const mrpt::config::CConfigFileBase& config,
		const std::string& section = "SENSOR");

	void run();

   private:
	// ----------------- ROS 2 params -----------------
	std::string out_rawlog_prefix_;

	std::string sensor_frame_id_ = "sensor";
	std::string publish_mrpt_obs_topic_ = "sensor_mrpt";
	std::string publish_topic_ = "sensor";
	// -----------------------------------------------

	mrpt::hwdrivers::CGenericSensor::Ptr sensor_;

	rclcpp::Publisher<mrpt_msgs::msg::GenericObservation>::SharedPtr
		obs_publisher_;

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
