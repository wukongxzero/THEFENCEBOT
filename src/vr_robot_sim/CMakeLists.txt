cmake_minimum_required(VERSION 3.8)
project(vr_robot_sim)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(vr_udp_publisher src/udp_publisher.cpp)
ament_target_dependencies(vr_udp_publisher rclcpp geometry_msgs)

add_executable(robot_controller src/robot_controller.cpp)
ament_target_dependencies(robot_controller rclcpp geometry_msgs)

install(TARGETS vr_udp_publisher robot_controller
    DESTINATION lib/${PROJECT_NAME})

ament_package()