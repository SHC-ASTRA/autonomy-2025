//=================================================================================================
// rover-Autonomy action Server Class Definition
// Last edited Jan 25, 2025
//=================================================================================================
// Hippocampus is responsible for memory and learning
//=================================================================================================
// Maintained by: Daegan Brown
// Number: 423-475-4384
// Email: daeganbrown03@gmail.com
//=================================================================================================
// Includes 
//=================================================================================================

// Prevents double compiling
#pragma once

// STD Includes
#include <string>                                       //Gosh I love strings in c++

// ROS2 Includes
#include "rclcpp/rclcpp.hpp"                            //Needed for ros2 cpp 
#include "rclcpp_action/rclcpp_action.hpp"              //Needed for actions, specifically
#include "clucky_interfaces/action/auto_command.hpp"    //Interfaces

//=================================================================================================
// Global Variables
//=================================================================================================

using AutoCommand = clucky_interfaces::action::AutoCommand;
using AutoCommandGoalHandle = rclcpp_action::ServerGoalHandle<AutoCommand>;
using namespace std::placeholders;


//=================================================================================================
// Class 
//=================================================================================================

class Hippocampus : public rclcpp::Node
{
    public:
        // Constructor
        Hippocampus();

        // Class Variables
        int mission_type;                                               //State
        double gps_lat_target, gps_long_target, target_radius, period;  //Inputs
        double gps_lat_current, gps_long_current;                       //Current Location
        std::string gps_string;                                         //Most recent GPS String
    private:
        // Response Behavior
        rclcpp_action::GoalResponse goal_callback(const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const AutoCommand::Goal> goal);
        
        // Cancel Behavior
        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<AutoCommandGoalHandle> goal_handle);

        // Seems redundant
        void handle_accepted_callback(
            const std::shared_ptr<AutoCommandGoalHandle> goal_handle);

        // Actual Action Server Code
        void execute_goal(
            const std::shared_ptr<AutoCommandGoalHandle> goal_handle);

    rclcpp_action::Server<AutoCommand>::SharedPtr auto_server_;
};