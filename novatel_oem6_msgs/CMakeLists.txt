cmake_minimum_required(VERSION 3.8)

project(novatel_oem6_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages:
  msg/BESTPOS.msg
  msg/INSCOVS.msg
  msg/INSPVAS.msg
  msg/IONUTC.msg
  msg/MARK2TIME.msg
  msg/MARKPOS.msg
  msg/MARKTIME.msg
  msg/RAWEPHEM.msg
  msg/RAWIMUS.msg
  msg/RXSTATUS.msg

  # Services:
  srv/SendNovatelCommand.srv
  
  # deps:
  DEPENDENCIES
    std_msgs
  ADD_LINTER_TESTS
)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

if(TARGET "${cpp_typesupport_target}")
  add_library(${PROJECT_NAME}_library INTERFACE)
  target_include_directories(${PROJECT_NAME}_library INTERFACE
  #"$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
  target_link_libraries(${PROJECT_NAME}_library INTERFACE
  "${cpp_typesupport_target}")

  install(
    TARGETS ${PROJECT_NAME}_library EXPORT export_${PROJECT_NAME}
  )

  # Export old-style CMake variables
  ament_export_include_directories("include/${PROJECT_NAME}")

  # Export modern CMake targets
  ament_export_targets(export_${PROJECT_NAME})
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()
