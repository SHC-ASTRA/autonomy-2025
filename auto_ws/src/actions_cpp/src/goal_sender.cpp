//=============================================================================
// rover-Autonomy Goal Sender
// Sends instructions to the server
// Last edited May 27, 2025
// Version: 2.0
//=============================================================================
// Includes
//=============================================================================
#include <iostream>
#include <chrono>

// ROS2 Comms
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_interfaces_pkg/action/auto_command.hpp"
#include "ros2_interfaces_pkg/msg/core_feedback.hpp"
#include "ros2_interfaces_pkg/msg/auto_feedback.hpp"

//=============================================================================
// Namespaces
//=============================================================================

using AutoCommand = ros2_interfaces_pkg::action::AutoCommand;
using AutoCommandGoalHandle = rclcpp_action::ClientGoalHandle<AutoCommand>;
using namespace std::placeholders;

//=============================================================================
// Class Definition
//=============================================================================

class AutoCommandClientNode : public rclcpp::Node
{
public:
    AutoCommandClientNode() : Node("auto_client")
    {
        auto_client_ =
            rclcpp_action::create_client<AutoCommand>(this, "navigate_rover");

        //---------------------------------------------------------------------
        // Parameters
        // Mission Type: Which Operation the rover performs
        // lat: Latitude target or general input 1
        // longi: Longitude target or general input 2
        // target radius: Target radius 
        // period: Rate of updating
        //---------------------------------------------------------------------
        this->declare_parameter("mission_type",-2);
        this->declare_parameter("lat",0.00);
        this->declare_parameter("longi",0.00);
        this->declare_parameter("radius",1.0);
        this->declare_parameter("period",0.8);
        


    }

    void send_goal()
    {
        // Wait for action server
        auto_client_->wait_for_action_server();

        // Create a goal
        auto goal = AutoCommand::Goal();

        // Get parameters into goal
        goal.mission_type = this->get_parameter("mission_type").as_int();
        goal.gps_lat_target = this->get_parameter("lat").as_double();
        goal.gps_long_target = this->get_parameter("longi").as_double();
        goal.target_radius = this->get_parameter("radius").as_double();
        goal.period = this->get_parameter("period").as_double();


        RCLCPP_INFO(get_logger(),
            "Starting goal client: mission=%ld, lat=%.3f, long=%.3f, radius=%.3f, period=%.3f",
            goal.mission_type, goal.gps_lat_target, goal.gps_long_target, goal.target_radius, goal.period);
        // Add callbacks
        auto options = rclcpp_action::Client<AutoCommand>::SendGoalOptions();
        options.feedback_callback =
            std::bind(&AutoCommandClientNode::feedback_callback, this, _1, _2);
        options.result_callback = 
            std::bind(&AutoCommandClientNode::goal_result_callback, this, _1);

        // Send Goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal");
        auto_client_->async_send_goal(goal, options);
        return;
    
    }

    void feedback_callback(
        AutoCommandGoalHandle::SharedPtr,
        const std::shared_ptr<const AutoCommand::Feedback> feedback)
    {
        std::stringstream ss;
        ss << "Next number in sequence received: ";
        RCLCPP_INFO(this->get_logger(), "%li", feedback->current_status);
    }
private:
    void timer_callback()
    {
        timer_->cancel();
    }

    void goal_result_callback(const AutoCommandGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED)
            RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        else if (status == rclcpp_action::ResultCode::ABORTED)
            RCLCPP_INFO(this->get_logger(), "Goal was aborted");
        
        int end = result.result->final_result;
        if (end == 0)
        {
            RCLCPP_INFO(this->get_logger(), "Goal ended with result '0', stopping client");
        }
        else if (end == 1)
        {
            RCLCPP_INFO(this->get_logger(), "Goal ended with result '1', restaring client");
            this->send_goal();
        }
        
    }

    rclcpp_action::Client<AutoCommand>::SharedPtr auto_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    AutoCommandGoalHandle::SharedPtr goal_handle;
};

//=============================================================================
// Main
//=============================================================================

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AutoCommandClientNode>();
    node->send_goal();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}