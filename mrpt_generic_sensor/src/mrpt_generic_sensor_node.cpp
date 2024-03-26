/* +------------------------------------------------------------------------+
   |                             mrpt_sensors                               |
   |                                                                        |
   | Copyright (c) 2017-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_sensors                      |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <rclcpp/rclcpp.hpp>

#include "mrpt_sensorlib/mrpt_sensorlib.h"

int main(int argc, char** argv)
{
    try
    {
        // Init ROS:
        rclcpp::init(argc, argv);

        auto node = std::make_shared<mrpt_sensors::GenericSensorNode>();
        RCLCPP_INFO(
            node->get_logger(), "About to init MrptGenericSensorNode...");
        node->init();
        RCLCPP_INFO(node->get_logger(), "Calling MrptGenericSensorNode::run()");
        node->run();

        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger(""),
            "Exception in mrpt_generic_sensor main(): " << e.what());
        return 1;
    }
}
