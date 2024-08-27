/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   |                                                                        |
   | Redistribution and use in source and binary forms, with or without     |
   | modification, are permitted provided that the following conditions     |
   | are met:                                                               |
   |                                                                        |
   | 1. Redistributions of source code must retain the above copyright      |
   |    notice, this list of conditions and the following disclaimer.       |
   |                                                                        |
   | 2. Redistributions in binary form must reproduce the above copyright   |
   |    notice, this list of conditions and the following disclaimer in the |
   |    documentation and/or other materials provided with the distribution.|
   |                                                                        |
   | 3. Neither the name of the copyright holder nor the names of its       |
   |    contributors may be used to endorse or promote products derived     |
   |    from this software without specific prior written permission.       |
   |                                                                        |
   | THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    |
   | "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      |
   | LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  |
   | A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   |
   | HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, |
   | SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       |
   | LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  |
   | DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  |
   | THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    |
   | (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  |
   | OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   |
   |                                                                        |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt/core/bits_math.h>  // square()
#include <mrpt/obs/CObservationIMU.h>
#include <mrpt/ros2bridge/image.h>

#include <rclcpp/rclcpp.hpp>

#include "mrpt_sensorlib/mrpt_sensorlib.h"

const char* node_name = "mrpt_sensor_imu_taobotics";
const char* sensorConfig = R""""(
# Configuration INI file block for mrpt-hwdrivers sensor.
# Read more online:
# https://www.mrpt.org/list-of-mrpt-apps/application-rawlog-grabber/
#
[SENSOR]
driver			= CTaoboticsIMU
process_rate	= ${PROCESS_RATE} // Hz

sensorLabel		= ${SENSOR_LABEL}

serialPort  = ${SERIAL_PORT}
sensorModel = ${SENSOR_MODEL}

pose_x       = ${SENSOR_POSE_X}      // (meters)
pose_y       = ${SENSOR_POSE_Y}
pose_z       = ${SENSOR_POSE_Z}
pose_yaw     = ${SENSOR_POSE_YAW}      // (deg)
pose_pitch   = ${SENSOR_POSE_PITCH}
pose_roll    = ${SENSOR_POSE_ROLL}

)"""";

int main(int argc, char** argv)
{
    try
    {
        // Init ROS:
        rclcpp::init(argc, argv);

        auto node =
            std::make_shared<mrpt_sensors::GenericSensorNode>(node_name);

        node->init(
            sensorConfig,
            {
                {"process_rate", "PROCESS_RATE", "500", false},
                {"sensor_label", "SENSOR_LABEL", "imu", false},
                {"serial_port", "SERIAL_PORT", "", true},
                {"sensor_model", "SENSOR_MODEL", "hfi-a9", false},

                {"sensor_pose_x", "SENSOR_POSE_X", "0", false},
                {"sensor_pose_y", "SENSOR_POSE_Y", "0", false},
                {"sensor_pose_z", "SENSOR_POSE_Z", "0", false},
                {"sensor_pose_yaw", "SENSOR_POSE_YAW", "0", false},
                {"sensor_pose_pitch", "SENSOR_POSE_PITCH", "0", false},
                {"sensor_pose_roll", "SENSOR_POSE_ROLL", "0", false},
            },
            {"SENSOR"});

        node->run();

        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(""),
            "Exception in " << node_name << " main(): " << e.what());
        return 1;
    }
}
