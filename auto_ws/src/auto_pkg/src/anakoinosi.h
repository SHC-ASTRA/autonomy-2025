//=================================================================================================
// rover-Autonomy action Server Listener/Subscriber Definition
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
#include <string>

// ROS2 Includes
#include "rclcpp/rclcpp.hpp"            // Needed for ROS2 cpp
#include "std_msgs/msg/string.hpp"      // 

// Local Includes
#include "anakoinosi.cpp"

//=================================================================================================
// Global Variables
//=================================================================================================

using std::placeholders::_1;

//=================================================================================================
// Class Definition
//=================================================================================================

class Anakoinosi : public rclcpp::Node
{
    public:
        // Constructor
        Anakoinosi();
    private:
        // Runs when /astra/core/feedback updates
        void topic_callback(const std_msgs::msg::String & msg) const;

        void topic_summon();

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};