//=================================================================================================
// rover-Autonomy Startup Main
// State machine for ASTRA rover
// Last edited Jan 25, 2025
//=================================================================================================
// Maintained by: Daegan Brown
// Number: 423-475-4384
// Email: daeganbrown03@gmail.com
//=================================================================================================
// Includes
//=================================================================================================

// ROS2 Includes
#include "rclcpp/rclcpp.hpp"                            //Needed for all cpp ROS files
#include "rclcpp_action/rclcpp_action.hpp"              //Needed for all ROS actions
#include "clucky_interfaces/action/auto_command.hpp"    //My interface

// Local Includes
#include "scn.cpp"

//=================================================================================================
// Main 
//=================================================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SCN>();
    node->send_goal(10, .8, .8, .8, .8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
