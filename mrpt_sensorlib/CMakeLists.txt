if ("$ENV{ROS_VERSION}" STREQUAL "2")
	cmake_minimum_required(VERSION 3.8)
	set(PACKAGE_ROS_VERSION 2)
	set(ROS_2 TRUE)
else()
	cmake_minimum_required(VERSION 3.1)
	set(PACKAGE_ROS_VERSION 1)
	set(ROS_1 TRUE)
endif()

project(mrpt_sensorlib)


# -----------------------------------------
# ROS 1
# -----------------------------------------
if (ROS_1)
	# Find catkin macros and# libraries
	# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	# is used, also find other catkin packages
	find_package(catkin REQUIRED COMPONENTS
		geometry_msgs
		message_generation
		mrpt_msgs
		roscpp
		sensor_msgs
		)

	###################################
	## catkin specific configuration ##
	###################################
	# The catkin_package macro generates cmake config files for your package
	# Declare things to be passed to dependent projects
	# INCLUDE_DIRS: uncomment this if you package contains header files
	# LIBRARIES: libraries you create in this project that dependent projects also need
	# CATKIN_DEPENDS: catkin_packages dependent projects also need
	# DEPENDS: system dependencies of this project that dependent projects also need
	catkin_package(
		INCLUDE_DIRS include
		LIBRARIES ${PROJECT_NAME}
		CATKIN_DEPENDS 
			geometry_msgs
			message_runtime
			roscpp
			mrpt_msgs
			sensor_msgs
		#DEPENDS mrpt-hwdrivers
	)

	find_package(mrpt-ros1bridge REQUIRED)
endif()

# -----------------------------------------
# ROS 2
# -----------------------------------------
if (ROS_2)
	# find dependencies
	find_package(ament_cmake REQUIRED)
	
	find_package(geometry_msgs REQUIRED)
	find_package(sensor_msgs REQUIRED)
	find_package(mrpt_msgs REQUIRED)

	find_package(mrpt-ros2bridge REQUIRED)
endif()

# Common ROS1/ROS2 deps:
find_package(mrpt-hwdrivers REQUIRED)

###########
## Build ##
###########

# Declare a cpp library
add_library(${PROJECT_NAME} SHARED
	src/${PROJECT_NAME}.cpp
	include/${PROJECT_NAME}/${PROJECT_NAME}.h)

# Specify additional locations of header files
# Your package locations should be listed before other locations

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

target_compile_definitions(${PROJECT_NAME} PUBLIC
  PACKAGE_ROS_VERSION=${PACKAGE_ROS_VERSION}
)

if (catkin_INCLUDE_DIRS)
	include_directories(${catkin_INCLUDE_DIRS})
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	# High level of warnings.
	target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
endif()

if (ROS_1)
	# Add cmake target dependencies of the executable/library
	# as an example, message headers may need to be generated before nodes
	add_dependencies(${PROJECT_NAME}
		${catkin_EXPORTED_TARGETS}
	)
endif()

if (ROS_1)
	target_link_libraries(${PROJECT_NAME} 
	PRIVATE
		${catkin_LIBRARIES}
		mrpt::ros1bridge
	PUBLIC
		mrpt::hwdrivers
	)
else()
	ament_target_dependencies(${PROJECT_NAME} 
		rclcpp
		geometry_msgs
		sensor_msgs
		mrpt_msgs
	)

	target_link_libraries(${PROJECT_NAME}
		mrpt::ros2bridge
		mrpt::hwdrivers
		)
endif()

#############
## Install ##
#############

# Mark executables and/or libraries for installation
if (ROS_2)
	set(CATKIN_PACKAGE_LIB_DESTINATION lib)
	set(CATKIN_PACKAGE_BIN_DESTINATION bin)
	set(CATKIN_PACKAGE_INCLUDE_DESTINATION include)
endif()

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executables and/or libraries for installation
if (ROS_2)
	ament_export_dependencies(mrpt-hwdrivers)

	# Export old-style CMake variables
	ament_export_include_directories("include/${PROJECT_NAME}")
	ament_export_libraries(${PROJECT_NAME})

	# Export modern CMake targets
	ament_export_targets(export_${PROJECT_NAME})

	ament_package()
endif()

#############
## Testing ##
#############
