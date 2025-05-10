//=================================================================================================
// rover-Autonomy action Server Class Implementation
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

// Local Includes
#include "hippocampus.h"

//=================================================================================================
// Global Variables
//=================================================================================================

using AutoCommand = clucky_interfaces::action::AutoCommand;
using AutoCommandGoalHandle = rclcpp_action::ServerGoalHandle<AutoCommand>;
using namespace std::placeholders;

//=================================================================================================
// Functions
//=================================================================================================

// Constructor 

Hippocampus::Hippocampus() : Node("auto_server")
{
    auto_server_ = rclcpp_action::create_server<AutoCommand>(
        this,
        "auto_command",
        std::bind(&Hippocampus::goal_callback, this, _1, _2),
        std::bind(&Hippocampus::cancel_callback, this, _1),
        std::bind(&Hippocampus::handle_accepted_callback, this, _1)

        
    );
    RCLCPP_INFO(this->get_logger(), "Action server has been started uWu");
}
// Response Behavior
rclcpp_action::GoalResponse Hippocampus::goal_callback(const rclcpp_action::GoalUUID &uuid,
    std::shared_ptr<const AutoCommand::Goal> goal)
{
    // Get rid of STDR outputs
    (void)uuid;
    
    // Validate Goal

    RCLCPP_INFO(this->get_logger(), "Recieved a goal");
    if (goal->mission_type <= 0)
    {
        RCLCPP_INFO(this->get_logger(), "Rejecting the goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(this->get_logger(), "Accepted the goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    // return rclcpp_action::GoalResponse::REJECT;
}

// Cancel Behavior
rclcpp_action::CancelResponse Hippocampus::cancel_callback(
    const std::shared_ptr<AutoCommandGoalHandle> goal_handle)
{
    // Get rid of STDR outputs
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

// Seems redundant
void Hippocampus::handle_accepted_callback(
    const std::shared_ptr<AutoCommandGoalHandle> goal_handle)
{
    // Pull request data from .action into global variables
    mission_type = goal_handle->get_goal()->mission_type;
    gps_lat_target = goal_handle->get_goal()->gps_lat_target;
    gps_long_target = goal_handle->get_goal()->gps_long_target;
    target_radius = goal_handle->get_goal()->target_radius;
    period = goal_handle->get_goal()->period;

    RCLCPP_INFO(this->get_logger(), "Executing the goal");
    execute_goal(goal_handle);
}

// Actual Action Server Code
void Hippocampus::execute_goal(
    const std::shared_ptr<AutoCommandGoalHandle> goal_handle)
{
    // I dont remember why this is needed
    rclcpp::Rate loop_rate(2.0/period);

    //*********************************************************************************
    //                               FAKE PLACEHOLDER
    //*********************************************************************************
    int counter = 0;
    
    for (int i = 0; i < mission_type; i++)
    {
        counter++;
        RCLCPP_INFO(this->get_logger(), "%d", counter);
        loop_rate.sleep();
    }
    //*********************************************************************************
    //                              END FAKE PLACEHOLDER
    //*********************************************************************************


    // Set final state and return result
    auto result = std::make_shared<AutoCommand::Result>();
    result->final_result = counter;
    goal_handle->succeed(result);
}