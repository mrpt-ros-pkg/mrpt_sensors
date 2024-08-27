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

#pragma once

//
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/obs/obs_frwds.h>
#include <tf2_ros/transform_broadcaster.h>

//
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

//

#include <mrpt_msgs/msg/generic_observation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

namespace mrpt_sensors
{
class GenericSensorNode : public rclcpp::Node
{
   public:
    explicit GenericSensorNode(
        const std::string& nodeName = "generic_sensor_node");

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
        const std::vector<std::string>& sections = {"SENSOR"});

    /// Initialize from the given configuration source:
    void init(
        const mrpt::config::CConfigFileBase& config,
        const std::vector<std::string>& sections = {"SENSOR"});

    void run();

    /// Once the observation is published as MRPT CObservation (if enabled),
    /// it will be published as a ROS message by process_observation(), unless
    /// a derived class implements this to handle it in a particular way.
    std::function<void(const mrpt::obs::CObservation::Ptr&)>
        custom_process_sensor;

    std::function<void()> init_sensor_specific;

    // Public members and variables for easy access from functors in
    // sensor-specific nodes
    template <class MSG_T, class PUB_T>
    void ensure_publisher_exists(
        PUB_T& pub, const std::string& topicSuffix = "")
    {
        if (!pub)
        {
            // QoS following REP-2003:
            // See: https://ros.org/reps/rep-2003.html
            pub = this->create_publisher<MSG_T>(
                publish_topic_ + topicSuffix, rclcpp::SystemDefaultsQoS());

            RCLCPP_INFO_STREAM(
                this->get_logger(),
                "Created publisher for topic: " << publish_topic_);
        }
    }

    std_msgs::msg::Header create_header(const mrpt::obs::CObservation& o);

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_publisher_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>
        images_publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_bc_;

    const auto sensor_frame_id() const { return sensor_frame_id_; }
    const auto publish_topic() const { return publish_topic_; }

   private:
    // ----------------- ROS 2 params -----------------
    std::string out_rawlog_prefix_;
    std::optional<mrpt::io::CFileGZOutputStream> out_rawlog_;

    std::string sensor_frame_id_ = "sensor";
    std::string robot_frame_id_ = "base_link";
    bool publish_sensor_pose_tf_ = true;
    double publish_sensor_pose_tf_minimum_period_ = 0.1;  // [s]

    std::string publish_mrpt_obs_topic_ = "sensor_mrpt";
    std::string publish_topic_ = "sensor";

    // -----------------------------------------------

    std::vector<mrpt::hwdrivers::CGenericSensor::Ptr> sensors_;

    double stamp_last_tf_publish_ = 0;

    rclcpp::Publisher<mrpt_msgs::msg::GenericObservation>::SharedPtr
        obs_publisher_;

    void process_observation(const mrpt::obs::CObservation::Ptr& o);

    void process(const mrpt::obs::CObservationGPS& o);
    void process(const mrpt::obs::CObservationIMU& o);
};
}  // namespace mrpt_sensors
