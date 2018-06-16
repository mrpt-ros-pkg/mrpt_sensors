cmake_minimum_required(VERSION 3.1)
project(mrpt_generic_sensor)
set (CMAKE_CXX_STANDARD 11)

## Find catkin macros and# libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS mrpt_sensorlib roscpp)

find_package(MRPT REQUIRED hwdrivers)
#message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")
if(NOT "${MRPT_VERSION}" VERSION_LESS "1.9.9")
	set (CMAKE_CXX_STANDARD 14)
endif()

if (CMAKE_COMPILER_IS_GNUCXX)
	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS mrpt_sensorlib
#  DEPENDS pcl mrpt  # opencv
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(${PROJECT_NAME}_node
#   mrpt_msgs_generate_messages_cpp
   ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${MRPT_LIBS}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
