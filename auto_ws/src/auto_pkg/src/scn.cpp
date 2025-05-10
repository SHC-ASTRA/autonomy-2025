//=================================================================================================
// rover-Autonomy Startup Client Class Implementation
// State machine for ASTRA rover
// Last edited Jan 25, 2025
//=================================================================================================
// SCN-suprachiasmatic nucleus, responsible for waking you up
// This action client is used as a starter, for testing, or for basestation to get autonomy 
// running. 
//=================================================================================================
// Maintained by: Daegan Brown
// Number: 423-475-4384
// Email: daeganbrown03@gmail.com
//=================================================================================================
// Includes 
//=================================================================================================

// Prevents double compiling
#pragma once

// ROS2 Includes
#include "rclcpp/rclcpp.hpp"                            //Needed for all cpp ROS files
#include "rclcpp_action/rclcpp_action.hpp"              //Needed for all ROS actions
#include "clucky_interfaces/action/auto_command.hpp"    //My interface

// Local Includes
#include "scn.h"

//=================================================================================================
// Using and Global Variables
//=================================================================================================

using AutoCommand = clucky_interfaces::action::AutoCommand;
using AutoCommandGoalHandle = rclcpp_action:: ClientGoalHandle<AutoCommand>;
using namespace std::placeholders;

//=================================================================================================
// Function Definitions
//=================================================================================================

// Constructor
SCN::SCN() : Node("auto_client_start")
{
    auto_client_start_ = 
        rclcpp_action::create_client<AutoCommand>(this, "auto_command");
}

// Sending Goal
void SCN::send_goal(int mission_type, double gps_lat_target, double gps_long_target, 
            double target_radius, double period)
{
    // Wait for action Server
            auto_client_start_->wait_for_action_server();

            // Get instructions 
            // std::cout << "Enter Mission Type: ";
            // std::cin >> mission_type;
            // std::cout << std::endl;

            // std::cout << "Enter GPS Target Latitude: ";
            // std::cin >> gps_lat_target;
            // std::cout << std::endl;

            // std::cout << "Enter GPS Target Longitude: ";
            // std::cin >> gps_long_target;
            // std::cout << std::endl;

            // std::cout << "Enter Target Radius: ";
            // std::cin >> mission_type;
            // std::cout << std::endl;


            // Create a goal
            
            auto goal = AutoCommand::Goal();
            goal.mission_type = mission_type;
            goal.gps_lat_target = gps_lat_target;
            goal.gps_long_target = gps_long_target;
            goal.target_radius = target_radius;
            goal.period = period;
            
            //Add callbacks
            auto options = rclcpp_action::Client<AutoCommand>::SendGoalOptions();
            options.result_callback = 
                std::bind(&SCN::goal_result_callback, this, _1);
            options.goal_response_callback =
                std::bind(&SCN::goal_response_callback, this, _1);


            // Send it
            RCLCPP_INFO(this->get_logger(), "Sending a goal");
            auto_client_start_->async_send_goal(goal, options);

            // Wait for callback
}

// Goal accpeted/rejected
void SCN::goal_response_callback(const AutoCommandGoalHandle::SharedPtr &goal_handle)
{
    if(!goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Goal got rejected");
    }
    else 
    {
        RCLCPP_INFO(this->get_logger(), "Goal got accepted");
    }
}

// Results
void SCN::goal_result_callback(const AutoCommandGoalHandle::WrappedResult &result)
{
    int exit_code = result.result->final_result;
    RCLCPP_INFO(this->get_logger(), "Result: %d", exit_code);
}