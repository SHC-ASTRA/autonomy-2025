//=================================================================================================
// rover-Autonomy Startup Client Class Definition
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

//=================================================================================================
// Using and Global Variables
//=================================================================================================

using AutoCommand = clucky_interfaces::action::AutoCommand;
using AutoCommandGoalHandle = rclcpp_action:: ClientGoalHandle<AutoCommand>;
using namespace std::placeholders;

//=================================================================================================
// Class Definition
//=================================================================================================

class SCN : public rclcpp::Node
{
    public:
        // Constructor
        SCN();

        // Sending Goals
        void send_goal(int mission_type, double gps_lat_target, double gps_long_target,
            double target_radius, double period);

    private:
        // Callback to know if accepted or rejected
        void goal_response_callback(const AutoCommandGoalHandle::SharedPtr &goal_handle);

        // Callback for result
        void goal_result_callback(const AutoCommandGoalHandle::WrappedResult &result);



    rclcpp_action::Client<AutoCommand>::SharedPtr auto_client_start_;
};