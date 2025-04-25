cmake_minimum_required(VERSION 3.8)
project(f1tenth)

if(POLICY CMP0009)
  cmake_policy(SET CMP0009 NEW)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Include JetGPIO if needed (leave in case your C++ files use it)
include_directories(include /usr/local/include)
find_library(JETGPIO_LIB jetgpio REQUIRED)
link_directories(/usr/local/lib)

# === C++ Executables ===

add_executable(pose_publisher src/pose_publisher.cpp)
ament_target_dependencies(pose_publisher
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS pose_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(tachometer_publisher src/tachometer_publisher.cpp)
ament_target_dependencies(tachometer_publisher
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(tachometer_publisher jetgpio)
install(TARGETS tachometer_publisher DESTINATION lib/${PROJECT_NAME})

add_executable(waypoint_follower src/waypointfollower.cpp)
ament_target_dependencies(waypoint_follower
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
)
install(TARGETS waypoint_follower DESTINATION lib/${PROJECT_NAME})

# === Python Scripts ===
install(PROGRAMS
  src/cmd_vel_to_odom.py
  src/jetson_gpio_control.py
  src/waypointfollower.py
  DESTINATION lib/${PROJECT_NAME}
)

# Launch and config folders (if you still use them)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# Optional testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
