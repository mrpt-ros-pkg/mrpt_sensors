/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "mrpt_sensorlib/mrpt_sensorlib.h"
#include <mrpt/config/CConfigFile.h>
#include <mrpt/system/filesystem.h>

#include <mrpt/obs/CObservationGPS.h>

// MRPT -> ROS bridge:
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/time.h>

using namespace mrpt::hwdrivers;
using namespace mrpt_sensors;

GenericSensorNode::GenericSensorNode() : Node("generic_sensor_node") {}

GenericSensorNode::~GenericSensorNode() {}

void GenericSensorNode::init()
{
	try
	{
		// Load parameters:
		this->declare_parameter("config_file", cfgfilename_);
		this->declare_parameter("config_section", cfg_section_);
		cfgfilename_ = this->get_parameter("config_file").as_string();
		cfg_section_ = this->get_parameter("config_section").as_string();
		mrpt::config::CConfigFile iniFile(cfgfilename_);

		// Call sensor factory:
		std::string driver_name =
			iniFile.read_string(cfg_section_, "driver", "", true);
		sensor_ = mrpt::hwdrivers::CGenericSensor::createSensorPtr(driver_name);
		if (!sensor_)
		{
			RCLCPP_ERROR_STREAM(
				this->get_logger(),
				"Sensor class name not recognized: " << driver_name);
			return;
		}

		// Load common & sensor-specific parameters:
		sensor_->loadConfig(iniFile, cfg_section_);

		// Initialize sensor:
		sensor_->initialize();

		// Custom init:
		this->init_sensor_specific();

		// Open rawlog file, if enabled:
		if (!out_rawlog_prefix_.empty())
		{
			// Build full rawlog file name:
			std::string rawlog_postfix = "_";
			mrpt::system::TTimeParts parts;
			mrpt::system::timestampToParts(mrpt::system::now(), parts, true);
			rawlog_postfix += mrpt::format(
				"%04u-%02u-%02u_%02uh%02um%02us", (unsigned int)parts.year,
				(unsigned int)parts.month, (unsigned int)parts.day,
				(unsigned int)parts.hour, (unsigned int)parts.minute,
				(unsigned int)parts.second);

			rawlog_postfix =
				mrpt::system::fileNameStripInvalidChars(rawlog_postfix);

			const std::string fil = out_rawlog_prefix_ + rawlog_postfix;
			// auto out_arch = archiveFrom(out_rawlog_);
			RCLCPP_INFO(
				this->get_logger(), "Writing rawlog to file: `%s`",
				fil.c_str());
		}
	}
	catch (const std::exception& e)
	{
		RCLCPP_ERROR_STREAM(
			this->get_logger(),
			"Exception in GenericSensorNode::init(): " << e.what());
		return;
	}
}

void GenericSensorNode::run()
{
	RCLCPP_INFO(
		this->get_logger(), "Starting run() at %.02f Hz",
		sensor_->getProcessRate());
	if (!sensor_)
	{
		RCLCPP_ERROR(
			this->get_logger(),
			"Aborting: sensor object was not properly initialized.");
		return;
	}
	rclcpp::Rate loop_rate(sensor_->getProcessRate());
	while (rclcpp::ok())
	{
		sensor_->doProcess();

		// Get new observations
		const CGenericSensor::TListObservations lstObjs =
			sensor_->getObservations();

		for (const auto& [t, obj] : lstObjs)
		{
			auto obs = std::dynamic_pointer_cast<mrpt::obs::CObservation>(obj);
			ASSERT_(obs);
			process_observation(obs);
		}

		rclcpp::spin_some(this->get_node_base_interface());
		loop_rate.sleep();
	}
}

void GenericSensorNode::process_observation(
	const mrpt::obs::CObservation::Ptr& o)
{
	if (auto oGPS = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(o);
		oGPS)
	{
		process(*oGPS);
	}
}

void GenericSensorNode::process(const mrpt::obs::CObservationGPS& o)
{
	if (!gps_publisher_)
	{
		gps_publisher_ =
			this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps", 1);
	}

	auto header = std_msgs::msg::Header();
	header.frame_id = "base_link";
	header.stamp = mrpt::ros2bridge::toROS(o.timestamp);

	auto msg = sensor_msgs::msg::NavSatFix();
	bool valid = mrpt::ros2bridge::toROS(o, header, msg);
	if (!valid) return;

	gps_publisher_->publish(msg);
}
