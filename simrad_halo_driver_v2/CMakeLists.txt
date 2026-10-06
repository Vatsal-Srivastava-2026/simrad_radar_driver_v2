cmake_minimum_required(VERSION 3.14.4)
project(simrad_halo_driver_v2)

set(ROS_DEPENDS
  ament_cmake
  ament_index_cpp
  builtin_interfaces
  rclcpp
  marine_radar_control_msgs
  marine_sensor_msgs
  nav_msgs
  rosbag2
  std_msgs
  tf2
)

set(EXTERNAL_DEPENDS
)

set(EXTERNAL_DEPENDS_LIB 
)


get_filename_component(PACKAGE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
message(STATUS "Creating Package: ${PACKAGE_NAME}")
project(${PACKAGE_NAME})  ## this package name is the name of the directory this cmake file is in


add_compile_definitions(CMAKE_PACKAGE_NAME=${PACKAGE_NAME})

# Default to C++17 for humble
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

###
###  Add dependencies
###

foreach(DEPEND ${ROS_DEPENDS} ${EXTERNAL_DEPENDS})
  find_package(${DEPEND} REQUIRED)
endforeach(DEPEND)

find_package(rosidl_default_generators REQUIRED)

set(INTERFACE_TARGET ${PROJECT_NAME})
rosidl_generate_interfaces(${INTERFACE_TARGET}
  "msg/HaloRadarSpoke.msg"
  "msg/HaloRadarSector.msg"
  "msg/HaloRadarEvent.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)
rosidl_get_typesupport_target(CPP_TYPESUPPORT_TARGET
  ${INTERFACE_TARGET} "rosidl_typesupport_cpp")


################
## Find files ##
################

file(GLOB HDRS CONFIGURE_DEPENDS
  "include/${PROJECT_NAME}/*.h"
)
file(GLOB SRCS CONFIGURE_DEPENDS
  "src/*.cpp"
  "src/*.hpp"
)
file(GLOB NODES CONFIGURE_DEPENDS
  "nodes/*.cpp"
)

include_directories(
  include
)

###########
## Build ##
###########

set(EXECUTABLES "")  # for installing later

message(STATUS "Building ${PROJECT_NAME} node:")

foreach(NODE_FILE ${NODES})
  get_filename_component(NODE_NAME ${NODE_FILE} NAME_WE)
  set(NODE_TARGET ${NODE_NAME}_executable)
  message(STATUS "  - ${NODE_NAME}")
  list(APPEND EXECUTABLES ${NODE_TARGET})
  add_executable(${NODE_TARGET} ${NODE_FILE} ${HDRS} ${SRCS})
  set_target_properties(${NODE_TARGET} PROPERTIES OUTPUT_NAME ${NODE_NAME})
  ament_target_dependencies(${NODE_TARGET} ${ROS_DEPENDS} ${EXTERNAL_DEPENDS_LIB})
  target_link_libraries(${NODE_TARGET} "${CPP_TYPESUPPORT_TARGET}")
  target_include_directories(${NODE_TARGET} PRIVATE "include/${PROJECT_NAME}/")
endforeach(NODE_FILE)


message(STATUS "Installing: ${EXECUTABLES}")
install(TARGETS
  ${EXECUTABLES}
  DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

#############
## testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_halo_driver
    test/test_halo_driver.cpp
    src/halo_radar.cpp
    src/halo_radar_node.cpp
    src/radar_sector_decoder.cpp
  )
  if(TARGET test_halo_driver)
    ament_target_dependencies(test_halo_driver ${ROS_DEPENDS})
    target_link_libraries(test_halo_driver "${CPP_TYPESUPPORT_TARGET}")
    target_include_directories(test_halo_driver PRIVATE "include/${PROJECT_NAME}/")
    set_tests_properties(test_halo_driver PROPERTIES TIMEOUT 45)
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)

ament_package()


#target_link_libraries(${PROJECT_NAME}
#  ${ROS_DEPENDS}
#)

#ament_target_dependencies(${PROJECT_NAME}
#  rclcpp
#  marine_radar_control_msgs
#  marine_sensor_msgs
#  rosbag2
#)

#target_link_libraries(${PROJECT_NAME} 
#  rclcpp::rclcpp
#  marine_radar_control_msgs
#  marine_sensor_msgs
#  rosbag2
#)


#add_executable(fix_angle_increments src/fix_angle_increments.cpp)
#ament_target_dependencies(fix_angle_increments
#  rclcpp
#  marine_radar_control_msgs
#  marine_sensor_msgs
#  rosbag2
#)
#target_link_libraries(fix_angle_increments 
#  rclcpp
#  marine_radar_control_msgs
#  marine_sensor_msgs
#  rosbag2
#)

#add_executable(fix_scan_times src/fix_scan_times.cpp)
#ament_target_dependencies(fix_scan_times
#  rclcpp
#  marine_radar_control_msgs
#  marine_sensor_msgs
#  rosbag2
#)
#target_link_libraries(fix_scan_times
#  rclcpp
#  marine_radar_control_msgs
#  marine_sensor_msgs
#  rosbag2
#)
