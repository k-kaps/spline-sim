cmake_minimum_required(VERSION 3.5)
project(limo_control)

# C++ settings:
set(CMAKE_CXX_STANDARD 17)
add_compile_options(-Wall -Wextra -g)

find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pure_pursuit_controller src/pure_pursuit_controller.cpp src/pid_controller.cpp)
target_include_directories(pure_pursuit_controller PUBLIC include)

ament_target_dependencies(pure_pursuit_controller rclcpp nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS pure_pursuit_controller DESTINATION lib/${PROJECT_NAME})
