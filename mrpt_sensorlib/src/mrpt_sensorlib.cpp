/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include "mrpt_sensorlib/mrpt_sensorlib.h"
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/obs/CObservationGPS.h>
#include <mrpt/serialization/CArchive.h>

// MRPT -> ROS bridge:
#include <mrpt/ros2bridge/gps.h>
#include <mrpt/ros2bridge/point_cloud2.h>
#include <mrpt/ros2bridge/image.h>
#include <mrpt/ros2bridge/time.h>

using namespace mrpt::hwdrivers;
using namespace mrpt_sensors;

GenericSensorNode::GenericSensorNode(const std::string& nodeName)
	: Node(nodeName)
{
}

GenericSensorNode::~GenericSensorNode() {}

void GenericSensorNode::init()
{
	try
	{
		std::string cfgfilename{"sensor.ini"}, cfg_section{"SENSOR1"};

		// Load parameters:
		this->declare_parameter("config_file", cfgfilename);
		this->declare_parameter("config_section", cfg_section);
		cfgfilename = this->get_parameter("config_file").as_string();
		cfg_section = this->get_parameter("config_section").as_string();

		mrpt::config::CConfigFile iniFile(cfgfilename);
		init(iniFile, cfg_section);
	}
	catch (const std::exception& e)
	{
		RCLCPP_ERROR_STREAM(
			this->get_logger(),
			"Exception in GenericSensorNode::init(): " << e.what());
		return;
	}
}

namespace
{
// https://stackoverflow.com/a/1494435/1631514
void text_replace(
	std::string& str, const std::string& oldStr, const std::string& newStr)
{
	std::string::size_type pos = 0u;
	while ((pos = str.find(oldStr, pos)) != std::string::npos)
	{
		str.replace(pos, oldStr.length(), newStr);
		pos += newStr.length();
	}
}
}  // namespace

void GenericSensorNode::init(
	const char* templateText, const std::vector<TemplateParameter>& rosParams,
	const std::string& section)
{
	using namespace std::string_literals;

	std::string text = templateText;

	// replace variables:
	for (const auto& p : rosParams)
	{
		if (!this->has_parameter(p.ros_param_name))
		{
			this->declare_parameter<std::string>(
				p.ros_param_name, p.default_value);
		}

		const std::string val =
			this->get_parameter(p.ros_param_name).as_string();

		if (p.required && val == p.default_value)
			THROW_EXCEPTION_FMT(
				"ROS 2 parameter '%s' was required for this sensor template "
				"but it was not defined.",
				p.ros_param_name.c_str());

		// replace: '${VARIABLE}' --> 'VALUE'
		text_replace(text, "${"s + p.template_variable + "}"s, val);
	}

	RCLCPP_DEBUG_STREAM(
		this->get_logger(), "init() with templated config block:\n"
								<< text);

	mrpt::config::CConfigFileMemory cfg(text);
	init(cfg, section);
}

void GenericSensorNode::init(
	const mrpt::config::CConfigFileBase& config, const std::string& section)
{
	try
	{
		// ----------------- Common ROS 2 params -----------------
		this->declare_parameter("out_rawlog_prefix", out_rawlog_prefix_);
		out_rawlog_prefix_ =
			this->get_parameter("out_rawlog_prefix").as_string();

		this->declare_parameter(
			"publish_mrpt_obs_topic", publish_mrpt_obs_topic_);
		publish_mrpt_obs_topic_ =
			this->get_parameter("publish_mrpt_obs_topic").as_string();

		this->declare_parameter("publish_topic", publish_topic_);
		publish_topic_ = this->get_parameter("publish_topic").as_string();

		this->declare_parameter("sensor_frame_id", sensor_frame_id_);
		sensor_frame_id_ = this->get_parameter("sensor_frame_id").as_string();

		// ----------------- End of common ROS 2 params -----------------

		// Call sensor factory:
		std::string driver_name =
			config.read_string(section, "driver", "", true);
		sensor_ = mrpt::hwdrivers::CGenericSensor::createSensorPtr(driver_name);
		if (!sensor_)
		{
			RCLCPP_ERROR_STREAM(
				this->get_logger(),
				"Sensor class name not recognized: " << driver_name);
			return;
		}

		// Load common & sensor-specific parameters:
		sensor_->loadConfig(config, section);

		// Initialize sensor:
		sensor_->initialize();

		// Custom init:
		if (init_sensor_specific) init_sensor_specific();

		// Open rawlog file, if enabled:
		if (!out_rawlog_prefix_.empty())
		{
			// Build full rawlog file name:
			std::string rawlog_postfix = "_";
			mrpt::system::TTimeParts parts;
			mrpt::system::timestampToParts(mrpt::Clock::now(), parts, true);
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
	// generic MRPT observation object:
	if (!publish_mrpt_obs_topic_.empty())
	{
		if (!obs_publisher_)
		{
			// QoS following REP-2003:
			// See: https://ros.org/reps/rep-2003.html
			obs_publisher_ =
				this->create_publisher<mrpt_msgs::msg::GenericObservation>(
					publish_mrpt_obs_topic_, rclcpp::SystemDefaultsQoS());

			RCLCPP_INFO_STREAM(
				this->get_logger(),
				"Created publisher for topic: " << publish_mrpt_obs_topic_);
		}

		mrpt_msgs::msg::GenericObservation msg;
		msg.header.frame_id = sensor_frame_id_;
		msg.header.stamp = mrpt::ros2bridge::toROS(o->timestamp);
		mrpt::serialization::ObjectToOctetVector(o.get(), msg.data);
		obs_publisher_->publish(msg);
	}

	// custom handling?
	if (custom_process_sensor)
	{
		custom_process_sensor(o);
		return;
	}

	// specific ROS messages:
	if (auto oGPS = std::dynamic_pointer_cast<mrpt::obs::CObservationGPS>(o);
		oGPS)
	{
		process(*oGPS);
	}
}

std_msgs::msg::Header GenericSensorNode::create_header(
	const mrpt::obs::CObservation& o)
{
	std_msgs::msg::Header header;
	header.frame_id = sensor_frame_id_;
	header.stamp = mrpt::ros2bridge::toROS(o.timestamp);
	return header;
}

void GenericSensorNode::process(const mrpt::obs::CObservationGPS& o)
{
	ensure_publisher_exists<sensor_msgs::msg::NavSatFix>(gps_publisher_);

#if 0
	std::stringstream ss;
	o.getDescriptionAsText(ss);
	RCLCPP_DEBUG_STREAM(this->get_logger(), ss.str());
#endif

	auto header = create_header(o);

	auto msg = sensor_msgs::msg::NavSatFix();
	bool valid = mrpt::ros2bridge::toROS(o, header, msg);
	if (!valid) return;

	gps_publisher_->publish(msg);
}
