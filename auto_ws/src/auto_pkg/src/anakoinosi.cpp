//=================================================================================================
// rover-Autonomy action Server Listener/Subscriber Implementation
// Last edited Jan 25, 2025
//=================================================================================================
// Anakoinosi - Communications
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
#include <memory>                       // DEBUG: This may be unneccesary

// ROS2 Includes
#include "rclcpp/rclcpp.hpp"            // Needed for ROS2 cpp
#include "std_msgs/msg/string.hpp"      // 

// ROS2 Communication Files
#include "ros2_interfaces_pkg/action/auto_command.hpp"  //contains action files and srv files
#include "ros2_interfaces_pkg/msg/core_feedback.hpp"

// Local Includes
#include "anakoinosi.h"                   // Class definition



//=================================================================================================
// Global Variables
//=================================================================================================

using std::placeholders::_1;

//=================================================================================================
// Functions
//=================================================================================================

// Constructor
Anakoinosi::Anakoinosi(): Node("core_listener")
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/astra/core/feedback", 10, std::bind(&Anakoinosi::topic_callback, this, _1));
    publisher_ = this->create_publisher<std_msgs::msg::String>(
        "/astra/core/feedback", 10);
}

void Anakoinosi::topic_callback(const std_msgs::msg::String & msg) const 
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}

void Anakoinosi::topic_summon() 
{
    auto message = std_msgs::msg::String();
    message.data = "Hell";
    RCLCPP_INFO(this->get_logger(), "Publishing '%s'", message.data.c_str());
    publisher_->publish(message);
}