//=============================================================================
//rover-Autonomy server client
//Sends instructions to the server
//Last edited May 20, 2025
//Version: 2.0
//=============================================================================
// Includes
//=============================================================================
#include <iostream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "astra_auto_interfaces/action/navigate_rover.hpp"


//=============================================================================
// Namespaces
//=============================================================================

using NavigateRover = astra_auto_interfaces::action::NavigateRover;
using NavigateRoverGoalHandle = rclcpp_action::ClientGoalHandle<NavigateRover>;
using namespace std::placeholders;

//=============================================================================
// Class Definition
//=============================================================================

class NavigateRoverClientNode : public rclcpp::Node 
{
public:
    NavigateRoverClientNode() : Node("navigate_rover_client") 
    {
        navigate_rover_client_ = 
            rclcpp_action::create_client<NavigateRover>(this, "navigate_rover");
    }

    void send_goal(int navigate_type, double gps_lat_target, 
        double gps_long_target, double target_radius, double period)
    {
        //Wait for the Action Server
        navigate_rover_client_->wait_for_action_server();

        // Create a goal
        auto goal = NavigateRover::Goal();
        goal.navigate_type = navigate_type;
        goal.gps_lat_target = gps_lat_target;
        goal.gps_long_target = gps_long_target;
        goal.target_radius = target_radius;
        goal.period = period;

        // Add callbacks
        auto options = rclcpp_action::Client<NavigateRover>::SendGoalOptions();
        options.feedback_callback =
            std::bind(&NavigateRoverClientNode::feedback_callback, this, _1, _2);
        options.result_callback = 
            std::bind(&NavigateRoverClientNode::goal_result_callback, this, _1);

        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending a goal");
        navigate_rover_client_->async_send_goal(goal, options);
        return;
    }

    void feedback_callback(
    NavigateRoverGoalHandle::SharedPtr,
    const std::shared_ptr<const NavigateRover::Feedback> feedback)
    {
        std::stringstream ss;
    ss << "Next number in sequence received: ";
    RCLCPP_INFO(this->get_logger(), "%li", feedback->current_status);
    }
private:

    void timer_callback()
    {
        //RCLCPP_INFO(this->get_logger(), "Canceled the goal");
        timer_->cancel();
    }


    // Callback to receive the results once the goal is done
    void goal_result_callback(const NavigateRoverGoalHandle::WrappedResult &result)
    {
        auto status = result.code;
        if (status == rclcpp_action::ResultCode::SUCCEEDED)
        {
            RCLCPP_INFO(this->get_logger(), "Succeeded");
        }
        else if (status == rclcpp_action::ResultCode::CANCELED)
        {
            RCLCPP_WARN(this->get_logger(), "Canceled");
        }
        
        
    }

    rclcpp_action::Client<NavigateRover>::SharedPtr navigate_rover_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    NavigateRoverGoalHandle::SharedPtr goal_handle;
};

//=============================================================================
// Main
//=============================================================================

int main(int argc, char **argv)
{
    int navigate_type;
    double gps_lat_target;
    double gps_long_target;
    double target_radius;
    double period = 0.8;

    //Which type of navigation. See astra_auto_server.cpp for a list of
    //options. 
    std::cout << "Input Type:" << std::endl;
    std::cin >> navigate_type; 
    std::cout << std::endl; 

    //The target latitude co-ordinate.
    //8 decimal places
    std::cout << "Target Latitude:" << std::endl;
    std::cin >> gps_lat_target; 
    std::cout << std::endl; 

    //The target longitude co-ordinate.
    //8 decimal places
    std::cout << "Target Longitude:" << std::endl;
    std::cin >> gps_long_target; 
    std::cout << std::endl; 

    //Target radius to search, for area searching;
    std::cout << "Target Radius:" << std::endl;
    std::cin >> target_radius;
    std::cout << std::endl;



    rclcpp::init(argc, argv);
    auto node = std::make_shared<NavigateRoverClientNode>(); 
    node->send_goal(navigate_type, gps_lat_target, gps_long_target, target_radius, 0.8);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}