
#include "mrpt_sensorlib/mrpt_sensorlib.h"

#include <mrpt/system/filesystem.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/serialization/CArchive.h>
#include <mrpt/config/CConfigFile.h>
using mrpt::config::CConfigFile;
#else
#include <mrpt/utils/CConfigFile.h>
using mrpt::utils::CConfigFile;
#endif

using namespace mrpt::hwdrivers;
using namespace mrpt_sensors;

GenericSensorNode::GenericSensorNode() {}

GenericSensorNode::~GenericSensorNode() {}

void GenericSensorNode::init(int argc, char** argv)
{
	try
	{
		// Load parameters:
		nhlocal_.getParam("config_file", cfgfilename_);
		nhlocal_.getParam("config_section", cfg_section_);
		CConfigFile iniFile(cfgfilename_);

		// Call sensor factory:
		std::string driver_name =
			iniFile.read_string(cfg_section_, "driver", "", true);
		sensor_ = mrpt::hwdrivers::CGenericSensor::createSensorPtr(driver_name);
		if (!sensor_)
		{
			ROS_ERROR_STREAM(
				"Sensor class name not recognized: " << driver_name);
			return;
		}

		// Load common & sensor specific parameters:
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

			// rawlog_postfix += dateTimeToString( now() );
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
			ROS_INFO("Writing rawlog to file: `%s`", fil.c_str());

			out_rawlog_.open(fil, rawlog_GZ_compress_level_);

			if (!out_rawlog_.is_open())
				ROS_ERROR("Error opening output rawlog for writing");
		}
	}
	catch (std::exception& e)
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
		CGenericSensor::TListObservations lstObjs;
		sensor_->getObservations(lstObjs);

		if (out_rawlog_.is_open())
		{
#if MRPT_VERSION >= 0x199
			auto out_arch = mrpt::serialization::archiveFrom(out_rawlog_);
#else
			auto& out_arch = out_rawlog_;
#endif
			for (const auto& o : lstObjs)
			{
				out_arch << *o.second;
			}
		}

		ros::spinOnce();
	}
}
