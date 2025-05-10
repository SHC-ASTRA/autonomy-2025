cmake_minimum_required(VERSION 3.8)
project(auto_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(clucky_interfaces REQUIRED)
find_package(ros2_interfaces_pkg REQUIRED)
find_package( OpenCV REQUIRED)

add_executable(auto_server src/starter_server.cpp)
ament_target_dependencies(auto_server rclcpp rclcpp_action clucky_interfaces)
target_include_directories(auto_server PUBLIC src/)

add_executable(auto_client_startup src/starter_client.cpp)
ament_target_dependencies(auto_client_startup rclcpp rclcpp_action clucky_interfaces)
target_include_directories(auto_client_startup PUBLIC src/)



install(TARGETS  
  auto_server
  auto_client_startup
  DESTINATION lib/${PROJECT_NAME}/
  )

install(DIRECTORY  
  launch
  DESTINATION share/${PROJECT_NAME}/
  )

ament_package()
