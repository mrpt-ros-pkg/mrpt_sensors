cmake_minimum_required(VERSION 3.5)
project(mrpt_generic_sensor)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mrpt_sensorlib REQUIRED)

find_package(mrpt_msgs REQUIRED)

find_package(mrpt-hwdrivers REQUIRED)
find_package(mrpt-ros2bridge REQUIRED)

message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")

###########
## Build ##
###########

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  mrpt::hwdrivers
  mrpt::ros2bridge
)

ament_target_dependencies(
	${PROJECT_NAME}_node
	"rclcpp"
  	"rclcpp_components"
  	"sensor_msgs"
  	"mrpt_sensorlib"
  	"mrpt_msgs"
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_node
	DESTINATION lib/${PROJECT_NAME}
)

#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}
#)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
