/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

//
#include <mrpt/config/CConfigFileBase.h>
#include <mrpt/hwdrivers/CGenericSensor.h>
#include <mrpt/obs/obs_frwds.h>
#include <tf2_ros/transform_broadcaster.h>

//
#include <functional>
#include <map>
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
        const std::string& section = "SENSOR");

    /// Initialize from the given configuration source:
    void init(
        const mrpt::config::CConfigFileBase& config,
        const std::string& section = "SENSOR");

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

   private:
    // ----------------- ROS 2 params -----------------
    std::string out_rawlog_prefix_;

    std::string sensor_frame_id_ = "sensor";
    std::string robot_frame_id_ = "base_link";
    bool publish_sensor_pose_tf_ = true;

    std::string publish_mrpt_obs_topic_ = "sensor_mrpt";
    std::string publish_topic_ = "sensor";

    // -----------------------------------------------

    mrpt::hwdrivers::CGenericSensor::Ptr sensor_;

    rclcpp::Publisher<mrpt_msgs::msg::GenericObservation>::SharedPtr
        obs_publisher_;

    void process_observation(const mrpt::obs::CObservation::Ptr& o);

    void process(const mrpt::obs::CObservationGPS& o);
};
}  // namespace mrpt_sensors
