/* mrpt_sensorlib ROS package
 *
 * Copyright 2021-2022, Jose Luis Blanco Claraco
 * License: BSD 3-Clause License
 */

#pragma once

#include <ros/ros.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <optional>
#include <mrpt/obs/obs_frwds.h>

namespace mrpt_sensorlib
{
/** Base class for all ROS nodes that interface an MRPT sensor.
 *  Provides the basic initialization of the sensor, parsing of incoming data,
 *  etc. Derived classes only have to worry about converting the observations
 *  to ROS msgs and expose any sensor-specific messages and services.
 */
class GenericSensorNode
{
   public:
	GenericSensorNode();
	virtual ~GenericSensorNode();

	/** Load parameters and establish communication with the sensor */
	virtual void init_from_config_file(int argc, char** argv);

	virtual void init_from_template_and_parameters(int argc, char** argv);

	/** Infinite loop for main(). */
	void run();

   protected:
	ros::NodeHandle nh_{};
	ros::NodeHandle nhlocal_{"~"};

	std::optional<std::string> cfgfilename_;
	std::shared_ptr<mrpt::config::CConfigFileBase> cfgfile_;
	std::string cfg_section_{"SENSOR1"};

	mrpt::hwdrivers::CGenericSensor::Ptr sensor_;
	std::string out_rawlog_prefix_;	 //!< If non-empty, write to rawlog
	mrpt::io::CFileGZOutputStream out_rawlog_;
	int rawlog_GZ_compress_level_{1};

	/** Uses cfgfile_ and cfg_section_ */
	virtual void internal_init();

	virtual void process_sensor_specific()
	{ /*do nothing by default*/
	}
	virtual void init_sensor_specific()
	{ /*do nothing by default*/
	}

	/** Process each observation and publish it to ROS topic */
	virtual void on_observation(const mrpt::obs::CObservation::Ptr& o);

	virtual void on_observation_velodyne(
		const mrpt::obs::CObservationVelodyneScan& o);
};

}  // namespace mrpt_sensorlib
