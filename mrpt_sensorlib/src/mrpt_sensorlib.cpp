/* mrpt_sensors ROS package
 *
 * Copyright 2021-2022, Jose Luis Blanco Claraco
 * License: BSD 3-Clause License
 */

#include "mrpt_sensorlib/mrpt_sensorlib.h"
#include <mrpt/system/filesystem.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/config/CConfigFileMemory.h>
#include <mrpt/io/vector_loadsave.h>  // file_get_contents()

#include <mrpt/ros1bridge/point_cloud2.h>
#include <mrpt/ros1bridge/time.h>
#include <mrpt/obs/CObservationVelodyneScan.h>
#include <mrpt/maps/CSimplePointsMap.h>

using namespace mrpt::hwdrivers;
using namespace mrpt_sensorlib;
using mrpt::config::CConfigFile;

GenericSensorNode::GenericSensorNode() {}

GenericSensorNode::~GenericSensorNode() {}

void GenericSensorNode::internal_init()
{
	try
	{
		ASSERT_(cfgfile_);

		// Call sensor factory:
		std::string driver_name =
			cfgfile_->read_string(cfg_section_, "driver", "", true);
		sensor_ = mrpt::hwdrivers::CGenericSensor::createSensorPtr(driver_name);
		if (!sensor_)
		{
			ROS_ERROR_STREAM(
				"Sensor class name not recognized: " << driver_name);
			return;
		}

		// Load common & sensor specific parameters:
		sensor_->loadConfig(*cfgfile_, cfg_section_);

		// Initialize sensor:
		sensor_->initialize();

		// Custom init:
		this->init_sensor_specific();

		// Open rawlog file, if enabled:
		if (!out_rawlog_prefix_.empty())
		{
			// Build full rawlog file name:

			std::string rawlog_postfix = "_";

			// rawlog_postfix += dateTimeToString( now() );
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
			ROS_INFO("Writing rawlog to file: `%s`", fil.c_str());

			out_rawlog_.open(fil, rawlog_GZ_compress_level_);

			if (!out_rawlog_.is_open())
				ROS_ERROR("Error opening output rawlog for writing");
		}
	}
	catch (const std::exception& e)
	{
		ROS_ERROR_STREAM(
			"Exception in GenericSensorNode::init(): " << e.what());
		return;
	}
}

void GenericSensorNode::init_from_config_file(	//
	[[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
	try
	{
		// Load parameters:
		std::string cfgFileName;
		nhlocal_.getParam("config_file", cfgFileName);
		cfgfilename_ = cfgFileName;

		cfgfile_ = std::make_shared<mrpt::config::CConfigFile>(*cfgfilename_);

		nhlocal_.getParam("config_section", cfg_section_);

		internal_init();
	}
	catch (const std::exception& e)
	{
		ROS_ERROR_STREAM(
			"Exception in GenericSensorNode::init(): " << e.what());
		return;
	}
}

void GenericSensorNode::init_from_template_and_parameters(	//
	[[maybe_unused]] int argc, [[maybe_unused]] char** argv)
{
	using namespace std::string_literals;

	try
	{
		// Load parameters:
		std::string templateFileName;
		nhlocal_.getParam("template_file", templateFileName);
		cfgfilename_ = templateFileName;

		ASSERT_FILE_EXISTS_(templateFileName);

		// Read template INI file and prepend all ROS node parameters:
		std::string cfgContents = mrpt::io::file_get_contents(templateFileName);

		// prepend ros node params:
		ros::V_string allParams;
		nhlocal_.getParamNames(allParams);
		for (const auto& paramName : allParams)
		{
			std::string value;
			if (!nhlocal_.getParam(paramName, value)) continue;

			cfgContents =
				"@define "s + paramName + " "s + value + "\n" + cfgContents;
		}

		ROS_INFO_STREAM("cfgContents:\n" << cfgContents);

		// Parse and run:
		cfgfile_ =
			std::make_shared<mrpt::config::CConfigFileMemory>(cfgContents);
		nhlocal_.getParam("template_file_section", cfg_section_);

		internal_init();
	}
	catch (const std::exception& e)
	{
		ROS_ERROR_STREAM(
			"Exception in GenericSensorNode::init(): " << e.what());
		return;
	}
}

/** Infinite loop for main(). */
void GenericSensorNode::run()
{
	ROS_INFO("Starting run() at %.02f Hz", sensor_->getProcessRate());
	if (!sensor_)
	{
		ROS_ERROR("Aborting: sensor object was not properly initialized.");
		return;
	}
	ros::Rate loop_rate(sensor_->getProcessRate());
	while (ros::ok())
	{
		sensor_->doProcess();

		// Get new observations
		CGenericSensor::TListObservations lstObjs = sensor_->getObservations();

		for (const auto& o : lstObjs)
		{
			if (const auto& obs =
					std::dynamic_pointer_cast<mrpt::obs::CObservation>(
						o.second);
				obs)
				on_observation(obs);
		}

		if (out_rawlog_.is_open())
		{
			auto out_arch = mrpt::serialization::archiveFrom(out_rawlog_);
			for (const auto& o : lstObjs)
			{
				out_arch << *o.second;
			}
		}

		ros::spinOnce();
	}
}

void GenericSensorNode::on_observation(const mrpt::obs::CObservation::Ptr& o)
{
	ASSERT_(o);

	if (const auto& obsVel =
			std::dynamic_pointer_cast<mrpt::obs::CObservationVelodyneScan>(o);
		obsVel)
	{
		on_observation_velodyne(*obsVel);
	}
	else
	{
		ROS_WARN(
			"Do not know how to publish observation '%s' of type '%s'",
			o->sensorLabel.c_str(), o->GetRuntimeClass()->className);
	}
}

void GenericSensorNode::on_observation_velodyne(
	const mrpt::obs::CObservationVelodyneScan& o)
{
	thread_local std::map<std::string, ros::Publisher> publishers;

	ROS_DEBUG("Received velodyne obs: %s", o.sensorLabel.c_str());

	// Create publisher on first use:
	if (publishers.count(o.sensorLabel) == 0)
	{
		publishers[o.sensorLabel] =
			nh_.advertise<sensor_msgs::PointCloud2>(o.sensorLabel, 20);
	}

	auto& pub = publishers[o.sensorLabel];

	if (o.point_cloud.size() == 0)
	{
		mrpt::obs::CObservationVelodyneScan::TGeneratePointCloudParameters p;
		p.minDistance = 0.2;

		const_cast<mrpt::obs::CObservationVelodyneScan&>(o)
			.generatePointCloud();
	}

	mrpt::maps::CSimplePointsMap pts;
	pts.loadFromVelodyneScan(o);

	std_msgs::Header hdr;
	hdr.frame_id = "base_link";
	hdr.stamp = mrpt::ros1bridge::toROS(o.timestamp);

	sensor_msgs::PointCloud2 velodyneCloud;
	bool ok = mrpt::ros1bridge::toROS(pts, hdr, velodyneCloud);
	ASSERT_(ok);

	pub.publish(velodyneCloud);
}
