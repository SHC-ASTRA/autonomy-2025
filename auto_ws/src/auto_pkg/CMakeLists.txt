cmake_minimum_required(VERSION 3.8)
project(auto_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(clucky_interfaces REQUIRED)

add_executable(auto_server src/server.cpp)
ament_target_dependencies(auto_server rclcpp rclcpp_action clucky_interfaces)

add_executable(auto_client_startup src/starter_client.cpp)
ament_target_dependencies(auto_client_startup rclcpp rclcpp_action clucky_interfaces)




install(TARGETS  
  auto_server
  auto_client_startup
  DESTINATION lib/${PROJECT_NAME}/
  )

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/action_client.py
  scripts/aruco_detector.py
  scripts/macula_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
