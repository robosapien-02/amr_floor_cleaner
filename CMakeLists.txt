cmake_minimum_required(VERSION 3.5)
project(homebot_0)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  include
)

add_executable(homebot src/homebot.cpp)
ament_target_dependencies(homebot
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

target_link_libraries(homebot Boost::system)

install(TARGETS
  homebot
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/laser_avoidance.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

