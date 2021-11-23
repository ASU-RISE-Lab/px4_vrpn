cmake_minimum_required(VERSION 3.5)
project(px4_vrpn_pubsub)

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
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)

add_executable(mocap_pubsub src/mocap_pubsub.cpp)
ament_target_dependencies(mocap_pubsub rclcpp px4_msgs std_msgs geometry_msgs tf2 tf2_ros Eigen3)


install(TARGETS
  mocap_pubsub
  DESTINATION lib/${PROJECT_NAME})

ament_package()
