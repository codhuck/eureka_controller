cmake_minimum_required(VERSION 3.16)
project(eureka_controller LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  control_msgs
  std_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
)

# Specify the required version of ros2_control
find_package(controller_manager )
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  eureka_controller
  SHARED
  hardware/hardware.cpp
  controllers/eureka_ackermann_controller.cpp
)
target_compile_features(eureka_controller PUBLIC cxx_std_17)
target_include_directories(eureka_controller PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/controllers/include>
$<INSTALL_INTERFACE:include/eureka_controller>
)
ament_target_dependencies(
  eureka_controller PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface eureka_controller.xml)
# Export controllers
pluginlib_export_plugin_description_file(controller_interface eureka_controller.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/eureka_controller
)
install(
  DIRECTORY controllers/include/
  DESTINATION include/eureka_controller
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/eureka_controller
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/eureka_controller
)
install(TARGETS eureka_controller
  EXPORT export_eureka_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(example_10_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_example_10_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_example_10_launch test/test_rrbot_launch.py)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_example_10 HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()