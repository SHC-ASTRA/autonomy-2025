//=============================================================================
//rover-Autonomy Server
//runs commands from the client
//Last edited May 27, 2025
//Version: 2.0
//=============================================================================
//INCLUDES
//=============================================================================

//C++ includes, normal
#include <memory>                           // 
#include <chrono>                           // 
#include <functional>                       // 
#include <string>                           // String type variable
#include <unistd.h>                         // usleep 
#include <stdio.h>
#include <algorithm>                        // Min
#include <cmath>                            // sin, cos, tan2
#include <utility>

//Made by Daegan Brown for ASTRA
// #include "pathfind.h"                       // My functions

//ROS2 includes
#include "rclcpp/rclcpp.hpp"                // General ROS2 stuff 
#include "rclcpp_action/rclcpp_action.hpp"  // ROS2 actions info
#include "rclcpp/subscription_options.hpp"  // ROS2 subsriber info
#include "std_msgs/msg/string.hpp"          // Message type for ROS2
#include "nav_msgs/msg/path.hpp"

// ROS2 Interfaces
#include "astra_msgs/action/auto_command.hpp"
#include "astra_msgs/msg/core_feedback.hpp"
#include "astra_msgs/msg/core_control.hpp"
#include "astra_msgs/msg/auto_feedback.hpp"
#include "astra_msgs/msg/auto_nav.hpp" 
#include "astra_msgs/msg/macula_feedback.hpp"

//=============================================================================
// Definitions
//=============================================================================

#define SECOND 1000000
#define FOCAL_RATIO 673.333313
#define HAMMER_RATIO 0.004523
#define BOTTLE_RATIO 0.002643

//=============================================================================
// Predeclarations
//=============================================================================


//Shorthands and other such things
using NavigateRover = astra_msgs::action::AutoCommand;
using NavigateRoverGoalHandle = rclcpp_action::ServerGoalHandle<NavigateRover>;
using namespace std::placeholders;


//Global Variables
double imu_bearing;                    
std::string gps_string;
bool cancel_request = false;

// Feedback
double current_heading;
double current_lat;
double current_long;
int sats;

// Macula
double macula_heading;
double macula_lat;
double macula_long;
double macula_range;
int object_id;
float x0_c, x1_c, x2_c, x3_c, y0_c, y1_c, y2_c, y3_c;


// Nav
double nav_x, nav_y, nav_z;



// Flags
bool canceled = 0;
bool anchorWait = 0;                // Is it waiting on /anchor/core/feedback?
bool arucoFound = 0;
bool hammerFound = 0;
bool bottleFound = 0;
bool navFail = 1;
bool holdMacula = 0;
bool navHold = 0;




//===========================================================================//
//= ROS2 Listener Nodes                                                     =//
//===========================================================================//


// Node for subcribing to topics
class NavigateRoverSubscriberNode : public rclcpp::Node 
{
public:
    
    NavigateRoverSubscriberNode() : Node("navigate_rover_subscriber")
    {
        subscriber_core_ = this->create_subscription<astra_msgs::msg::CoreFeedback>(
            "/core/feedback", 10, std::bind(&NavigateRoverSubscriberNode::core_callback, this, _1));
        subscriber_anchor_ = this->create_subscription<std_msgs::msg::String>(
            "/anchor/core/feedback", 10, std::bind(&NavigateRoverSubscriberNode::anchor_callback, this, _1));
        // subscriber_macula_ = this->create_subscription<astra_msgs::msg::MaculaFeedback>(
        //     "/auto/macula", 10, std::bind(&NavigateRoverSubscriberNode::macula_callback, this, _1));
        
        
        
        

    }
private:
    //=======================================================================//
    //= Subscriber Topic Callback                                           =//
    //=======================================================================//
    void core_callback(const astra_msgs::msg::CoreFeedback & msg) 
    {
        current_heading = msg.orientation;
        current_lat = msg.gps_lat;
        current_long = msg.gps_long;
        sats = msg.gps_sats;
     
        RCLCPP_INFO(this->get_logger(), "Recieved Orientation: '%f' ", current_heading);
        RCLCPP_INFO(this->get_logger(), "Recieved Latitude: '%f' ", current_lat);
        RCLCPP_INFO(this->get_logger(), "Recieved Longitude: '%f' ", current_long);

    }

    void anchor_callback(const std_msgs::msg::String & msg)
    {
        if (anchorWait == 0 || holdMacula)
            return;
        if (msg.data == "can_relay_fromvic,core,drivemeters_done")
        {
            anchorWait = 0;
        }
    }


    //=======================================================================//
    //= Internal Variables                                                  =//
    //=======================================================================//
    // bool detected;
    // int object_id;
    // float x0, x1, x2, x3, y0, y1, y2, y3;

    rclcpp::Subscription<astra_msgs::msg::CoreFeedback>::SharedPtr subscriber_core_;
    rclcpp::Subscription<astra_msgs::msg::MaculaFeedback>::SharedPtr subscriber_macula_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_anchor_;

};

//===========================================================================//
//= ROS2 Server Node                                                         =//
//===========================================================================//
// Node for the action server
class NavigateRoverServerNode : public rclcpp::Node 
{
public:
    //=======================================================================//
    //= Constructor                                                         =//
    //=======================================================================//
    NavigateRoverServerNode() : Node("navigate_rover_server"), count_(0) 
    {
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        //Creating action
        navigate_rover_server_ = rclcpp_action::create_server<NavigateRover>(
                this,
                "navigate_rover",
                std::bind(&NavigateRoverServerNode::goal_callback, this, _1, _2),
                std::bind(&NavigateRoverServerNode::cancel_callback, this, _1),
                std::bind(&NavigateRoverServerNode::handle_accepted_callback, this, _1),
                rcl_action_server_get_default_options(),
                cb_group_
            );
        RCLCPP_INFO(this->get_logger(), "Action server has been started");
        
        // Publisher for Core Control
        publisher_core = this->create_publisher<astra_msgs::msg::CoreControl>(
            "/core/control", 10);

        // Publisher to send information directly to anchor
        publisher_anchor = this->create_publisher<std_msgs::msg::String>(
            "/anchor/relay", 10);
        
        // Publisher to contact Nav2
        publisher_nav = this->create_publisher<astra_msgs::msg::AutoNav>(
            "/auto/nav", 10);
        
    }
    
private:
    //=======================================================================//
    //= Internal Variables                                                  =//
    //=======================================================================//
    // Mission Info
    int mission_type;
    double target_lat;
    double target_long;
    double target_radius;
    double period;

    // Derived info 
    float target_bearing;
    float distance_remaining;

    //=======================================================================//
    //= ROS2 Action Standard Functions                                      =//
    //=======================================================================//
    
    //-------------------------------------------------------------------------
    // Callback of recieved Goal
    // This function handles accepting/rejecting goal
    // TODO: Add a check to make sure within 2km range
    //-------------------------------------------------------------------------
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const NavigateRover::Goal> goal)
    {
        //to get rid of startup warnings
        (void)uuid;
        
        RCLCPP_INFO(this->get_logger(), "Recieved Goal");
        // Invalid mission types
        if (goal->mission_type > 15 || goal->mission_type < -10)
        {   
            publish_info("Rejected Goal! Out of bounds!");
            return rclcpp_action::GoalResponse::REJECT;
        }
        // Acceptable, then proceed
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    //-------------------------------------------------------------------------
    // Cancelling the Goal
    // This function runs when a cancel request is receieved
    //-------------------------------------------------------------------------
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<NavigateRoverGoalHandle> goal_handle)
    {
        // Get rid of stdr output at build
        (void)goal_handle;
        // Stop rover
        set_motors(0);

        // Feedback
        publish_info("Recieved Goal Cancel Request but WONT FUCKIN DO IT");
        cancel_request = true;
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    //-------------------------------------------------------------------------
    // Handle Callback
    // This function is called when a goal is accepted
    // Currently logs that, stores goal variables, and then executes goal
    //-------------------------------------------------------------------------
    void handle_accepted_callback(
        const std::shared_ptr<NavigateRoverGoalHandle> goal_handle)
    {
        // First save goal variables
        mission_type = goal_handle->get_goal()->mission_type;
        target_lat = goal_handle->get_goal()->gps_lat_target;
        target_long = goal_handle->get_goal()->gps_long_target;
        target_radius = goal_handle->get_goal()->target_radius;
        period = goal_handle->get_goal()->period;

        // Execute the goal
        publish_info("Goal was accepted, now executing");
        execute_goal(goal_handle);
    }
    //-------------------------------------------------------------------------
    // Execution of the Goal
    // This is *the* function that handles the goal. The big boy. 
    //-------------------------------------------------------------------------
    void execute_goal(
        const std::shared_ptr<NavigateRoverGoalHandle> goal_handle)
    {
        // Create result variable
        int t_result;
        auto result = std::make_shared<NavigateRover::Result>();

        // Set rate
        rclcpp::Rate loop_rate(1.0/period);

        // Set LED to RED
        set_led(1);                         // Red

        // Check status
        if (goal_handle->is_canceling())
        {
            publish_info("Goal was canceled!");
            publish_info("Cancel happened right before switch in execute_goal().");
            set_led(0);

        }
        //---------------------------------------------------------------------
        // This switch statement is determined by the launch parameters.
        // Negative mission types are debug types. 
        //---------------------------------------------------------------------
        switch (mission_type) {
            //-----------------------------------------------------------------
            // Case 0: 
            // Rover will wait 4 seconds, change LED to green, wait 4 seconds, 
            // to blue, then to red, then turn to bearing target, then stop.
            //-----------------------------------------------------------------
            case 0: 
                usleep(1 * SECOND);
                set_led(2);
                usleep(1 * SECOND);
                set_led(3);
                usleep(1 * SECOND);
                set_led(1);
                // Update bearing and orient to it
                
                set_bearing();
                orient(target_bearing);
                result->final_result = 1;
                break;
            //-----------------------------------------------------------------
            // Case 1: GNSS Legacy
            // Rover will point to target, drive forward 3 seconds, then 
            // repeat until within target radius or goal is canceled.
            //-----------------------------------------------------------------
            case 1: 
                publish_info("Spinning up Legacy Nav");
                legacy_nav();
                break;
            
            case -5:
                publish_info("Started mission -5: Serial Orient");
                refresh();
                serial_orient(target_bearing);
                break;
            case -6:
                publish_info("Started mission -6: Manual Orient");
                refresh();
                // manual_orient(target_bearing);
                break;
            
        }
        if (goal_handle->is_canceling())
        {
            publish_info("Goal was canceled!");
            publish_info("Cancel happened during switch.");
            set_led(0);
        }


        // Send Result
        result->final_result = t_result;
        goal_handle->succeed(result);
        publish_info("Goal Succeeded!");

        // Set LED to blink green
        for (int i = 0; i < 5; i++)
        {
            set_led(2);
            usleep(0.5 * SECOND);
            set_led(0);
            usleep(0.5 * SECOND);
        }
        set_led(2);

    }
    
    //=======================================================================//
    //= Direct Rover Control Commands                                       =//
    //=======================================================================//
    // These functions directly control the rover, either through 
    // /core/control or through /anchor/relay
    //-------------------------------------------------------------------------
    // Set motors
    // Currently three states to control motors:
    // 0: Stopped
    // 1: Going Forward
    // 2: GOing Backward
    // 3: Going Forward Slowly
    //-------------------------------------------------------------------------
    void set_motors(int state)
    {
        publish_info("Running Function: set_motors()");
        auto message = astra_msgs::msg::CoreControl();

        // Stop
        if (state == 0)
        {
            publish_info("Stopping motors!");
            message.left_stick = 0;
            message.right_stick = 0;
            message.max_speed = 70;
            message.brake = false;
            message.turn_to_enable = false;
            publisher_core->publish(message);
        }
        // Go Forwards
        else if (state == 1)
        {
            publish_info("Going Forward!");
            message.left_stick = 1;
            message.right_stick = 1;
            message.max_speed = 90;
            message.brake = false;
            message.turn_to_enable = false;
            publisher_core->publish(message);
        }
        // Go Backwards
        else if (state == 2)
        {
            publish_info("Going Backwards!");
            message.left_stick = .7;
            message.right_stick = .7;
            message.max_speed = 70;
            message.brake = false;
            message.turn_to_enable = false;
            publisher_core->publish(message);
        }
        // Go forwards, slowly
        else if (state == 3)
        {
            publish_info("Going Forward Slowly!");
            message.left_stick = .7;
            message.right_stick = .7;
            message.max_speed = 70;
            message.brake = false;
            message.turn_to_enable = false;
            publisher_core->publish(message);
        }
        // Warn, stop! Invalid input
        else 
        {
            publish_warn("Invalid motor state!");
            message.left_stick = 0;
            message.right_stick = 0;
            message.max_speed = 70;
            message.brake = false;
            message.turn_to_enable = false;
            publisher_core->publish(message);
            publish_warn("Stopped motors, future behaviour may be undefined!");
        }


    }

    //-------------------------------------------------------------------------
    // Orient To
    // Input a bearing, and the function will continue until the rover is 
    // within 2 degree of that target bearing
    //-------------------------------------------------------------------------
    
    void orient(float bearing)
    {
        publish_info("Running Function: orient()");
        // astra_msgs::msg::CoreControl message;
        // message.turn_to_enable = true;
        // message.turn_to = bearing;
        // message.turn_to_timeout = 10;
        // std::string info_str = "Turning to face " + std::to_string(bearing);

        std_msgs::msg::String balls = std_msgs::msg::String();
        int direction = (int)bearing;
        // balls.data = "\ncan_relay_tovic,core,41,350,1\n";
        balls.data = "can_relay_tovic,core,41," + std::to_string(direction) + ",10\n";
        publisher_anchor->publish(balls);
        // publisher_core->publish(message);
        usleep(10 * SECOND);
        // rclcpp::Rate rate(10); // 10 Hz => 100 ms per iteration
        // int max_iters = 50;    // 50 * 100 ms => 5 seconds
        // while (rclcpp::ok() && max_iters--)
        // {
        // publish_info(info_str.c_str());+
        // publisher_core->publish(message);

        // // Let callbacks run so current_heading can be updated by subscriber:
        // rclcpp::spin_some(this->get_node_base_interface());

        // if (std::abs(current_heading - bearing) < 5) {
        //     publish_info("Orientation within tolerance");
        //     return;
        // }
        // rate.sleep();
        // }
        std::string scommand = "Orienting to: " + std::to_string(bearing);
        publish_info(scommand);
            
        
    }

    // Old serial 
    void serial_orient(float bearing)
    {
        publish_info("Running Function: serial_orient()");
        auto command = std_msgs::msg::String();
        std::string scommand = "auto,turningTo,10," + std::to_string(bearing);
        command.data = scommand;
        usleep(10 * SECOND);

    }

    //-------------------------------------------------------------------------
    // Legacy Navigate
    // Orients, then goes towards point relative to distance left
    // Upgraded legacy URC 2024 code
    //-------------------------------------------------------------------------
    void legacy_nav()
    {
        publish_info("Running Function: legacy_nav()");
        publish_info("Begining Legacy point-to-point navigation");
        while (!(check_target()))
        {
            refresh();
            orient(target_bearing);
            if (distance_remaining >= 15)
                drive_time(10.0);
            else if (distance_remaining >= 6)
                drive_time(6.0);
            else if (distance_remaining >= 3)
                drive_time(3.0);
            else 
                drive_time(2.0);
        }
    }

    //-------------------------------------------------------------------------
    // Set LED
    // Changes LED color
    // 0 = OFF
    // 1 = RED
    // 2 = GREEN
    // 3 = BLUE
    //-------------------------------------------------------------------------
    void set_led(int color)
    {
        publish_info("Running Function: set_led()");
        auto command = std_msgs::msg::String();
        


        switch (color) {
            case 0:
                publish_info("Turning off LED");
                command.data = "led_set,0,0,0\n";
                break;
            case 1:
                publish_info("Turning LED red");
                command.data = "led_set,255,0,0\n";
                break;
            case 2:
                publish_info("Turning LED Green");
                command.data = "led_set,0,255,0\n";
                break;
            case 3:
                publish_info("Turning LED Blue");
                command.data = "led_set,0,0,255\n";
                break;
            default:
                publish_info("Turning off LED");
                publish_warn("Recieved unknown LED command. Turning off LED and proceeding");
                command.data = "led_set,0,0,0\n";
            break;
            
        }
        publisher_anchor->publish(command);
    }
    
    //-------------------------------------------------------------------------
    // Drive Time
    // This function sends to /core/control to run the rover forward for an 
    // inputted float of time. 
    //-------------------------------------------------------------------------
    void drive_time(float duration)
    {
        publish_info("Running Function: drive_time()");
        set_distance_remaining();
        set_bearing();
        // if (distance_remaining < 5 || macula_range < 5)
        // {
            // Account for safety timeout
            if (duration > 1) {
                for (int i = 0; i < std::floor(duration); i++) {
                    set_motors(3);
                    usleep(1 * SECOND);
                }
            }
            // Run the decimal point time (e.g., if duration == 1.5, already ran for 1 seconds, now run for 0.5 secs)
            if (duration == (int)duration) {
                set_motors(3);
                usleep((duration - std::floor(duration)) * SECOND);
            }
            set_motors(0);
        // }
        // else 
        // {
            // set_motors(1);
            // usleep(duration * SECOND);
            // set_motors(0);
        // }

    }

    //=======================================================================//
    //= Refresh Functions                                                   =//
    //=======================================================================//
    // These funtions refresh local variables or wait for input to refresh
    // said variables
    
    //-------------------------------------------------------------------------
    // Refresh
    // This function refreshes internal gps, bearing, bearing target, and 
    // distance remaining variables.
    //-------------------------------------------------------------------------
    void refresh()
    {
        publish_info("Running Function: refresh()");
        
        set_bearing();
        set_distance_remaining();
        publish_info("Data refreshed!");
    }

    //-------------------------------------------------------------------------
    // Check Target
    // This functions checks if we are within 1.5 meters of target.
    // It returns true if we are, otherwise returns false. 
    //-------------------------------------------------------------------------
    bool check_target()
    {
        publish_info("Running Function: check_target()");
        
        if ((abs(current_lat - target_lat) <= 0.000018) && \
            ((abs(current_long - target_long) <= 0.000018)))
        {
            publish_info("Within Target Bounds!");
            return true;
        }
        else
            return false;
    }


    //-------------------------------------------------------------------------
    // Set Target Bearing
    // This function resets the internal target_bearing variable to needed one, 
    // based on up-to-date gps data
    //-------------------------------------------------------------------------
    void set_bearing()
    {
        publish_info("Running Function: set_bearing()");
        double X, Y, neededHeading;
        double deltaLong = target_long - current_long;
        double deg2rad = (3.141592/180);
        double rad2deg = (180/3.141592);
        int i_neededHeading;

        X = ( std::cos(deg2rad * target_lat) * std::sin(deg2rad * deltaLong));
        Y = ( std::cos(deg2rad * current_lat) * std::sin( deg2rad * target_lat))\
            - (std::sin(deg2rad * current_lat) * std::cos(deg2rad * target_lat) * \
            std::cos(deg2rad * deltaLong));
        neededHeading = (rad2deg * atan2(X,Y)) + 360;

        i_neededHeading = neededHeading;
        i_neededHeading = i_neededHeading % 360;
        target_bearing = (float)i_neededHeading;

        // Macula too
        X = ( std::cos(deg2rad * macula_lat) * std::sin(deg2rad * deltaLong));
        Y = ( std::cos(deg2rad * current_lat) * std::sin( deg2rad * macula_lat))\
            - (std::sin(deg2rad * current_lat) * std::cos(deg2rad * macula_lat) * \
            std::cos(deg2rad * deltaLong));
        neededHeading = (rad2deg * atan2(X,Y)) + 360;

        i_neededHeading = neededHeading;
        i_neededHeading = i_neededHeading % 360;
        macula_heading = (float)i_neededHeading;


    }

    //-------------------------------------------------------------------------
    // Set Distance remaining
    // This function resets the interanl distance_remaining variable to the 
    // correct one, based on up-to-date gps data
    //-------------------------------------------------------------------------
    void set_distance_remaining()
    {
        publish_info("Running Function: set_distance_remaining()");
        double deg2rad = (3.141592/180);
        double deltaLat = deg2rad * target_lat - current_lat;
        double deltaLong = deg2rad * target_long - current_long;
        double a;
        double c;
        double d;
        int R = 6371000;    // Earth Radius

        // Haversine formula
        a = (std::sin(deltaLat / 2) * std::sin(deltaLat / 2)) + \
            (std::cos(deg2rad * current_lat) * std::cos(deg2rad * target_lat) * \
            (std::sin(deltaLong / 2) * std::sin(deltaLat / 2)));
        c = 2 * atan2(sqrt(a), sqrt(1-a));
        d = R * c;

        distance_remaining = d;
    }

    //=======================================================================//
    //= Heavy Duty Calc                                                     =//
    //=======================================================================//
    // Some heavy calculation functions

    //=======================================================================//
    //= ROS2 Shortcuts                                                      =//
    //=======================================================================//

    //-------------------------------------------------------------------------
    // Publish Debug, Info, Warn, Error, Fatal
    //-------------------------------------------------------------------------
    
    void publish_debug(const char * msg)
    {
        RCLCPP_DEBUG(this->get_logger(), msg);
    }
    void publish_info(const char * msg)
    {
        RCLCPP_INFO(this->get_logger(), msg);
    }
    void publish_warn(const char * msg)
    {
        RCLCPP_WARN(this->get_logger(), msg);
    }
    void publish_error(const char * msg)
    {
        RCLCPP_ERROR(this->get_logger(), msg);
    }
    void publish_fatal(const char * msg)
    {
        RCLCPP_FATAL(this->get_logger(), msg);
    }

    //=======================================================================//
    //= ROS2 Declarations                                                   =//
    //=======================================================================//
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_anchor;
    rclcpp::Publisher<astra_msgs::msg::CoreControl>::SharedPtr publisher_core;
    rclcpp::Publisher<astra_msgs::msg::AutoNav>::SharedPtr publisher_nav;
    size_t count_;
    rclcpp_action::Server<NavigateRover>::SharedPtr navigate_rover_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
};

//====================================================================================
// Main
//====================================================================================
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node1 = std::make_shared<NavigateRoverServerNode>(); 
    auto node2 = std::make_shared<NavigateRoverSubscriberNode>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node1);
    executor.add_node(node2);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
