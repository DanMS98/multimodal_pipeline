cmake_minimum_required(VERSION 3.8)
project(sensor_sync)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Include directories
include_directories(include)

# Generate the custom message
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SyncedSensors.msg"
  DEPENDENCIES sensor_msgs builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime sensor_msgs builtin_interfaces std_msgs)

install(PROGRAMS
  sensor_sync/sync_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
