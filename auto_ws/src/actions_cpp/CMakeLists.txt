cmake_minimum_required(VERSION 3.8)
project(actions_cpp VERSION 1.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILTE_COMMANDS true)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(astra_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
# find_package(realsense2_camera REQUIRED)

find_package( OpenCV REQUIRED )


include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(navigate_rover_server src/astra_auto_server.cpp)
#target_link_libraries( navigate_rover_server ${OpenCV_LIBS} )
ament_target_dependencies(navigate_rover_server rclcpp rclcpp_action std_msgs astra_msgs nav_msgs)
target_link_libraries( navigate_rover_server ${OpenCV_LIBS} opencv_highgui)

add_executable(navigate_rover_client src/astra_auto_client.cpp)
ament_target_dependencies(navigate_rover_client rclcpp rclcpp_action astra_msgs)

add_executable(goal src/goal_sender.cpp)
ament_target_dependencies(goal rclcpp rclcpp_action std_msgs astra_msgs)

add_executable(navigation src/simple_server.cpp)
ament_target_dependencies(navigation rclcpp rclcpp_action std_msgs astra_msgs)

install(TARGETS 
  navigate_rover_server
  navigate_rover_client
  goal
  navigation
  
  DESTINATION lib/${PROJECT_NAME}/
)
install ( 
  DIRECTORY ${OpenCV_INCLUDE_DIRS}
  DESTINATION lib/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()
