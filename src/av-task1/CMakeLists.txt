cmake_minimum_required(VERSION 3.8)
project(av-task1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(av-position src/av-position.cpp)
add_executable(av-speed src/av-speed.cpp)
add_executable(av-steering-control src/av-steering-control.cpp)


ament_target_dependencies(av-position rclcpp geometry_msgs)
ament_target_dependencies(av-speed rclcpp std_msgs)
ament_target_dependencies(av-steering-control geometry_msgs rclcpp std_msgs)

install(TARGETS 
  av-position
  av-speed
  av-steering-control
  DESTINATION lib/${PROJECT_NAME})


ament_package()
