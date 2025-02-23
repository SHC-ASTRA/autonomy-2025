//*************************************************************************************************
// rover-Autonomy Server
// State machine for ASTRA rover
// Last edited Sept 9, 2024
//*************************************************************************************************
// Maintained by: Daegan Brown
// Number: 423-475-4384
// Email: daeganbrown03@gmail.com
//*************************************************************************************************
// Includes
//*************************************************************************************************

// STD Includes
#include <string>                                       //Gosh I love strings in c++

// ROS2 Includes
#include "rclcpp/rclcpp.hpp"                            //Needed for ros2 cpp 
#include "rclcpp_action/rclcpp_action.hpp"              //Needed for actions, specifically
#include "clucky_interfaces/action/auto_command.hpp"    //Interfaces

// Local Includes
#include "hippocampus.h"                              //Class files
#include "hippocampus.cpp"


//*************************************************************************************************
// Global Variables
//*************************************************************************************************

using AutoCommand = clucky_interfaces::action::AutoCommand;
using AutoCommandGoalHandle = rclcpp_action::ServerGoalHandle<AutoCommand>;
using namespace std::placeholders;


//*************************************************************************************************
// Main
//*************************************************************************************************

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Hippocampus>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}