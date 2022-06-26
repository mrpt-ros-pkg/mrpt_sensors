if ("$ENV{ROS_VERSION}" STREQUAL "2")
	cmake_minimum_required(VERSION 3.8)
	set(PACKAGE_ROS_VERSION 2)
	set(ROS_2 TRUE)
else()
	cmake_minimum_required(VERSION 3.1)
	set(PACKAGE_ROS_VERSION 1)
	set(ROS_1 TRUE)
endif()

project(mrpt_generic_sensor)

# -----------------------------------------
# ROS 1
# -----------------------------------------
if (ROS_1)
	# Find catkin macros and# libraries
	# if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
	# is used, also find other catkin packages
	find_package(catkin REQUIRED COMPONENTS
		roscpp
		mrpt_sensorlib
	)

	###################################
	## catkin specific configuration ##
	###################################
	catkin_package(
		CATKIN_DEPENDS 
			mrpt_sensorlib
			roscpp
	)

	find_package(mrpt-ros1bridge REQUIRED)
endif()

# -----------------------------------------
# ROS 2
# -----------------------------------------
if (ROS_2)
	# find dependencies
	find_package(ament_cmake REQUIRED)
	find_package(mrpt_sensorlib REQUIRED)
	find_package(mrpt_msgs REQUIRED)
	
	find_package(mrpt-ros2bridge REQUIRED)
endif()

# Common ROS1/ROS2 deps:
find_package(mrpt-hwdrivers REQUIRED)

###########
## Build ##
###########

if (catkin_INCLUDE_DIRS)
	include_directories(${catkin_INCLUDE_DIRS})
endif()

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_compile_definitions(${PROJECT_NAME}_node PUBLIC
  PACKAGE_ROS_VERSION=${PACKAGE_ROS_VERSION}
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	# High level of warnings.
	target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

if (ROS_1)
	# Add cmake target dependencies of the executable/library
	# as an example, message headers may need to be generated before nodes
	add_dependencies(${PROJECT_NAME}_node
	${catkin_EXPORTED_TARGETS}
	)
endif()

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   mrpt::hwdrivers
)
if (ROS_1)
	target_link_libraries(${PROJECT_NAME}_node
		${catkin_LIBRARIES}
		mrpt::ros1bridge
	)
else()
	ament_target_dependencies(${PROJECT_NAME}_node
		mrpt_sensorlib
	)

	target_link_libraries(${PROJECT_NAME}_node
		mrpt::ros2bridge
		#mrpt_sensorlib #??
		)
endif()


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html


# Mark executables and/or libraries for installation
if (ROS_2)
	set(CATKIN_PACKAGE_LIB_DESTINATION lib)
	set(CATKIN_PACKAGE_BIN_DESTINATION bin)
	set(CATKIN_PACKAGE_INCLUDE_DESTINATION include)

	#ament_export_dependencies() # ???
endif()

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
