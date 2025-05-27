//=============================================================================
// rover-Autonomy Goal Sender
// Sends instructions to the server
// Last edited May 26, 2025
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
    }

    void send_goal()
    {

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
        
        // result.result.
        
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