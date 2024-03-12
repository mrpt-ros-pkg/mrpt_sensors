cmake_minimum_required(VERSION 3.8)
project(mrpt_sensorlib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mrpt_msgs REQUIRED)
find_package(mrpt-ros2bridge REQUIRED)
find_package(mrpt-hwdrivers REQUIRED)

message(STATUS "MRPT_VERSION: ${mrpt-hwdrivers_VERSION}")

###########
## Build ##
###########

# Declare a cpp library
add_library(${PROJECT_NAME}
  src/mrpt_sensorlib.cpp
  include/mrpt_sensorlib/mrpt_sensorlib.h
)

target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
  mrpt::hwdrivers
  mrpt::ros2bridge
)


#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

#############
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  mrpt-ros2bridge
  mrpt-hwdrivers
  mrpt_msgs
#  tf2
)


#find_package(ament_cmake_gtest REQUIRED)
#ament_add_gtest(${PROJECT_NAME}-test test/test_pose_cov_ops.cpp)

# Export old-style CMake variables
#ament_export_include_directories("include/${PROJECT_NAME}")
#ament_export_libraries(${PROJECT_NAME}_export HAS_LIBRARY_TARGET)

# Export modern CMake targets
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_cpplint REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
