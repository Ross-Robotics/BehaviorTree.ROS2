cmake_minimum_required(VERSION 3.16)
project(behaviortree_ros2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

set(THIS_PACKAGE_DEPS
  rclcpp
  rclcpp_action
  ament_index_cpp
  btcpp_ros2_interfaces
  generate_parameter_library
)

foreach(PACKAGE ${THIS_PACKAGE_DEPS})
  find_package(${PACKAGE} REQUIRED)
endforeach()

find_path(
  BEHAVIORTREE_CPP_INCLUDE_DIR
  NAMES behaviortree_cpp/bt_factory.h
  PATHS
    /opt/ros/humble/include
  REQUIRED
)

find_library(
  BEHAVIORTREE_CPP_LIBRARY
  NAMES behaviortree_cpp
  PATHS
    /opt/ros/humble/lib
    /opt/ros/humble/lib/${CMAKE_LIBRARY_ARCHITECTURE}
    /opt/ros/humble/lib/aarch64-linux-gnu
  REQUIRED
)

add_library(BT::behaviortree_cpp SHARED IMPORTED)

set_target_properties(
  BT::behaviortree_cpp
  PROPERTIES
    IMPORTED_LOCATION "${BEHAVIORTREE_CPP_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${BEHAVIORTREE_CPP_INCLUDE_DIR}"
)

message(STATUS
  "Using BehaviorTree.CPP library: ${BEHAVIORTREE_CPP_LIBRARY}"
)

#
# Parameters
#

generate_parameter_library(
  bt_executor_parameters
  src/bt_executor_parameters.yaml
)

#
# Library
#

add_library(
  ${PROJECT_NAME}
  src/bt_ros2.cpp
  src/bt_utils.cpp
  src/tree_execution_server.cpp
)

target_include_directories(
  ${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  ament_index_cpp::ament_index_cpp
  ${btcpp_ros2_interfaces_TARGETS}
  BT::behaviortree_cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  PRIVATE
  bt_executor_parameters
)

#
# Install
#

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    ${PROJECT_NAME}
    bt_executor_parameters
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#
# Exports
#

ament_export_targets(
  ${PROJECT_NAME}Targets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${THIS_PACKAGE_DEPS}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()