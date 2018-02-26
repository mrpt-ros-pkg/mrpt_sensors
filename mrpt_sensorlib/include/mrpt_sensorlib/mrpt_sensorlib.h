
#pragma once

#include <ros/ros.h>
#include <mrpt/version.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#if MRPT_VERSION >= 0x199
#include <mrpt/io/CFileGZOutputStream.h>
using mrpt::io::CFileGZOutputStream;
#else
#include <mrpt/utils/CFileGZOutputStream.h>
using mrpt::utils::CFileGZOutputStream;
#endif

namespace mrpt_sensors
{
/** Base class for all ROS nodes that interface an MRPT sensor.
 *  Provides the basic initialization of the sensor, parsing of incoming data,
 *  etc. Derived classes only have to worry about converting the observations
 *  to ROS msgs and expose any sensor-specific messages and services.
 */
class GenericSensorNode
{
   protected:
	ros::NodeHandle nh_{};
	ros::NodeHandle nhlocal_{"~"};
	std::string cfgfilename_{"sensor.ini"}, cfg_section_{"SENSOR1"};
	mrpt::hwdrivers::CGenericSensor::Ptr sensor_;
	std::string out_rawlog_prefix_;  //!< If non-empty, write to rawlog
	CFileGZOutputStream out_rawlog_;
	int rawlog_GZ_compress_level_{1};

	virtual void process_sensor_specific()
	{ /*do nothing by default*/
	}
	virtual void init_sensor_specific()
	{ /*do nothing by default*/
	}

   public:
	GenericSensorNode();
	virtual ~GenericSensorNode();

	/** Load parameters and establish communication with the sensor */
	virtual void init(int argc, char** argv);

	/** Infinite loop for main(). */
	void run();
};

}  // namespace mrpt_sensors
