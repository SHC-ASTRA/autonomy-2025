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

//Made by Daegan Brown for ASTRA
// #include "pathfind.h"                       // My functions

//ROS2 includes
#include "rclcpp/rclcpp.hpp"                // General ROS2 stuff
#include "rclcpp_action/rclcpp_action.hpp"  // ROS2 actions info
#include "rclcpp/subscription_options.hpp"  // ROS2 subsriber info
#include "std_msgs/msg/string.hpp"          // Message type for ROS2

//openCV shenanigans
#include <opencv2/opencv.hpp>                   //
#include <opencv2/core.hpp>                     //
#include <opencv2/aruco.hpp>                    //
#include <opencv2/videoio.hpp>                  //
#include <opencv2/highgui.hpp>                  //
#include <opencv2/objdetect/aruco_detector.hpp> //
#include <opencv2/calib3d.hpp>                  //

// ROS2 Interfaces
#include "ros2_interfaces_pkg/action/auto_command.hpp"
#include "ros2_interfaces_pkg/msg/core_feedback.hpp"
#include "ros2_interfaces_pkg/msg/core_control.hpp"
#include "ros2_interfaces_pkg/msg/auto_feedback.hpp"
#include "ros2_interfaces_pkg/msg/auto_nav.hpp" 
#include "ros2_interfaces_pkg/msg/macula_feedback.hpp"

//=============================
// Definitions
//=============================

#define SECOND 1000000
#define FOCAL_RATIO 700.000
//=============================================================================
// Predeclarations
//=============================================================================


//Shorthands and other such things
using NavigateRover = ros2_interfaces_pkg::action::AutoCommand;
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



// Flags
bool canceled = 0;
bool coreWait = 1;                  // Is it waiting on /core/feedback?
bool anchorWait = 0;                // Is it waiting on /anchor/core/feedback?
bool arucoFound = 0;
bool hammerFound = 0;
bool bottleFound = 0;
bool navFail = 1;
bool holdMacula = 0;




//===========================================================================//
//= ROS2 Listener Nodes                                                     =//
//===========================================================================//


// Node for subcribing to topics
class NavigateRoverSubscriberNode : public rclcpp::Node 
{
public:
    
    NavigateRoverSubscriberNode() : Node("navigate_rover_subscriber")
    {
        subscriber_core_ = this->create_subscription<ros2_interfaces_pkg::msg::CoreFeedback>(
            "/core/feedback", 10, std::bind(&NavigateRoverSubscriberNode::core_callback, this, _1));
        subscriber_anchor_ = this->create_subscription<std_msgs::msg::String>(
            "/anchor/core/feedback", 10, std::bind(&NavigateRoverSubscriberNode::anchor_callback, this, _1));
        subscriber_macula_ = this->create_subscription<ros2_interfaces_pkg::msg::MaculaFeedback>(
            "/auto/macula", 10, std::bind(&NavigateRoverSubscriberNode::macula_callback, this, _1));
        
        
        

    }
private:
    //=======================================================================//
    //= Subscriber Topic Callback                                           =//
    //=======================================================================//
    void core_callback(const ros2_interfaces_pkg::msg::CoreFeedback & msg) 
    {
        if (coreWait)
        {
            RCLCPP_INFO(this->get_logger(), "Recieved Core Feedback!");
            coreWait = 0;
        }
        current_heading = msg.orientation;
        current_lat = msg.gps_lat;
        current_long = msg.gps_long;
        sats = msg.gps_sats;

        RCLCPP_DEBUG(this->get_logger(), "Recieved Orientation: '%f' ", current_heading);
        RCLCPP_DEBUG(this->get_logger(), "Recieved Latitude: '%f' ", current_lat);
        RCLCPP_DEBUG(this->get_logger(), "Recieved Longitude: '%f' ", current_long);
        RCLCPP_DEBUG(this->get_logger(), "With '%d' satellites", sats);
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

    void macula_callback(const ros2_interfaces_pkg::msg::MaculaFeedback & msg)
    {
        if (!msg.detected || holdMacula == 1)
        {
            if (holdMacula == 0)
            {
                hammerFound = 0;
                bottleFound = 0;
                arucoFound = 0;
            }
            return;
        }    
        RCLCPP_INFO(this->get_logger(), "Spotted target in frame");


        // Detect What
        if (msg.object_id == 51)
        {
            RCLCPP_INFO(this->get_logger(), "Hammer Detected!");
            hammerFound = 1;

        }
        else if (msg.object_id == 51)
        {
            RCLCPP_INFO(this->get_logger(), "Water Bottle Detected!");
            bottleFound = 1;
        }
        else 
        {
            RCLCPP_INFO(this->get_logger(), "AruCo Tag number '%d' detected", msg.object_id);
            arucoFound = 1;
        }
        
        // Save data
        // detected = msg.detected;
        object_id = msg.object_id;
        x0_c = msg.x0;
        x1_c = msg.x1;
        x2_c = msg.x2;
        x3_c = msg.x3;
        y0_c = msg.y0;
        y1_c = msg.y1;
        y2_c = msg.y2;
        y3_c = msg.y3;

        // Hold until cleared by server
        holdMacula = 1;
    }


    //=======================================================================//
    //= Internal Variables                                                  =//
    //=======================================================================//
    // bool detected;
    // int object_id;
    // float x0, x1, x2, x3, y0, y1, y2, y3;

    rclcpp::Subscription<ros2_interfaces_pkg::msg::CoreFeedback>::SharedPtr subscriber_core_;
    rclcpp::Subscription<ros2_interfaces_pkg::msg::MaculaFeedback>::SharedPtr subscriber_macula_;
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
        publisher_core = this->create_publisher<ros2_interfaces_pkg::msg::CoreControl>(
            "/core/control", 10);

        // Publisher to send information directly to anchor
        publisher_anchor = this->create_publisher<std_msgs::msg::String>(
            "/anchor/relay", 10);
        
        // Publisher to contact Nav2
        publisher_nav = this->create_publisher<ros2_interfaces_pkg::msg::AutoNav>(
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
        if (goal->mission_type > 15 || goal->mission_type < -2)
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
            publish_debug("Cancel happened right before switch in execute_goal().");
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
                {
                    usleep(4 * SECOND);
                    set_led(2);
                    usleep(4 * SECOND);
                    set_led(3);
                    usleep(4 * SECOND);
                    set_led(1);
                    // Update bearing and orient to it
                    set_bearing();
                    orient(target_bearing);
                        result->final_result = 1;
                }
                break;
            //-----------------------------------------------------------------
            // Case 1:
            // Rover will point to target, drive forward 3 seconds, then 
            // repeat until within target radius or goal is canceled.
            //-----------------------------------------------------------------
            case 1:
                while (!(check_target()) && !(goal_handle->is_canceling()) && !(navFail))
                {

                }
                if (navFail)
                {
                    legacy_nav();
                }
                break;

            //-----------------------------------------------------------------
            // Case -1: 
            // Needs to run with Macula to test rangefinding. 
            // Will publish found range 10 times, then exit with result.
            //-----------------------------------------------------------------
            case -1:
                publish_debug("Started mission -1");
                for (int i = 0; i < 11; i++)
                {
                    while (holdMacula = 0);
                    range_aruco();
                    RCLCPP_INFO(this->get_logger(), "Found Macula Range: '%f'", macula_range);
                    RCLCPP_INFO(this->get_logger(), "Found Macula Lat: '%f'", macula_lat);
                    RCLCPP_INFO(this->get_logger(), "Found Macula Long: '%f'", macula_long);
                    RCLCPP_INFO(this->get_logger(), "Found Macula Heading: '%f'", macula_heading);
                }
                t_result = 0;
                break;
            //-----------------------------------------------------------------
            // Case -2: 
            // Runs case -1 in reverse, using target radius as the range. 
            //-----------------------------------------------------------------
            case -2:
                publish_debug("Started mission -2");
                while (!holdMacula);
                calibrate_camera();
                t_result = 0;
                break;
        }
        if (goal_handle->is_canceling())
        {
            publish_info("Goal was canceled!");
            publish_debug("Cancel happened during switch.");
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
    //-------------------------------------------------------------------------
    void set_motors(int state)
    {
        publish_debug("Running Function: set_motors()");
        auto message = ros2_interfaces_pkg::msg::CoreControl();

        // Stop
        if (state == 0)
        {
            publish_info("Stopping motors!");
            message.left_stick = 0;
            message.right_stick = 0;
            publisher_core->publish(message);
        }
        // Go Forwards
        else if (state == 1)
        {
            publish_info("Going Forward!");
            message.left_stick = .7;
            message.right_stick = .7;
            publisher_core->publish(message);
        }
        // Go Backwards
        else if (state == 2)
        {
            publish_info("Going Backwards!");
            message.left_stick = -0.7;
            message.right_stick = -0.7;
            publisher_core->publish(message);
        }
        // Warn, stop! Invalid input
        else 
        {
            publish_warn("Invalid motor state!");
            message.left_stick = 0;
            message.right_stick = 0;
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
        publish_debug("Running Function: orient()");
        // Create and populate message and 
        auto message = ros2_interfaces_pkg::msg::CoreControl();
        message.turn_to_enable = false;
        message.turn_to = bearing;
        message.turn_to_timeout = 10;
        std::string msg = "Turning to face " + std::to_string(bearing); 
        const char * c_msg = msg.c_str();
        
        
        do {
            publish_debug(c_msg);

            publisher_core->publish(message);
            for (int i = 0; i < 11; i++)
            {
                confirm_core();
                if (abs(current_heading - bearing) <= 2)
                    break;
                usleep(SECOND);
            }
        } while (abs(current_heading - bearing) <= 2);
            
        
    }

    //-------------------------------------------------------------------------
    // Legacy Navigate
    // Orients, then goes towards point relative to distance left
    // Upgraded legacy URC 2024 code
    //-------------------------------------------------------------------------
    void legacy_nav()
    {
        publish_debug("Running Function: legacy_nav()");
        publish_info("Begining Legacy point-to-point navigation");
        while (!(check_target()) && !canceled)
        {
            refresh();
            orient(target_bearing);
            if (distance_remaining >= 15)
                drive_meters(10);
            else if (distance_remaining >= 10)
                drive_meters(5);
            else if (distance_remaining >= 5)
                drive_meters(1);
        }
    }

    //-------------------------------------------------------------------------
    // Legacy AruCo Navigate
    // Orients, then goes towards aruco relative to distance left
    // Upgraded legacy URC 2024 code
    //-------------------------------------------------------------------------
    void legacy_aruco_nav()
    {
        publish_debug("Running Function: legacy_aruco_nav()");
        publish_info("Begining Legacy AruCo navigation");
        while (!(check_macula_target()) && !canceled)
        {
            refresh();
            orient(macula_heading);
            drive_meters(1);
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
        publish_debug("Running Function: set_led()");
        auto command = std_msgs::msg::String();
        
        switch (color) {
            case 0:
            publish_info("Turning off LED");
            command.data = "led_set,0,0,0";
            break;
            case 1:
            publish_info("Turning LED red");
            command.data = "led_set,255,0,0";
            break;
            case 2:
            publish_info("Turning LED Green");
            command.data = "led_set,0,255,0";
            break;
            case 3:
            publish_info("Turning LED Blue");
            command.data = "led_set,0,0,255";
            break;
            default:
            publish_info("Turning off LED");
            publish_warn("Recieved unknown LED command. Turning off LED and proceeding");
            command.data = "led_set,0,0,0";
            break;
            
        }
        publisher_anchor->publish(command);
    }
    
    //-------------------------------------------------------------------------
    // Drive Meters
    // This function sends to /anchro/relay a command to drive the rover
    // x meters forward (backwards if negative)
    //-------------------------------------------------------------------------
    void drive_meters(float meters)
    {
        publish_debug("Running Function: drive_meters()");
        auto command = std_msgs::msg::String();
        std::string scommand = "driveMeters," + std::to_string(meters);
        command.data = scommand.c_str();

        anchorWait = 1;
        while(anchorWait == 1)
        {
            usleep(0.25 * SECOND);
            publisher_anchor->publish(command);
        }
        // Stop
        set_motors(0);
        publish_debug("Went the distance");
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
        publish_debug("Running Function: refresh()");
        confirm_core();
        set_bearing();
        set_distance_remaining();
        publish_debug("Data refreshed!");
    }

    //-------------------------------------------------------------------------
    // Confirm Core data (gps, bearing)
    // This function waits until we recieved new message on /core/feedback
    //-------------------------------------------------------------------------

    void confirm_core()
    {
        publish_debug("Running Function: confirm_core()");
        publish_debug("Waiting for /core/feedback");
        coreWait = 1;
        while (coreWait);
        publish_debug("Recieved /core/feedback");
    }

    //-------------------------------------------------------------------------
    // Check Target
    // This functions checks if we are within 1.5 meters of target.
    // It returns true if we are, otherwise returns false. 
    //-------------------------------------------------------------------------
    bool check_target()
    {
        publish_debug("Running Function: check_target()");
        confirm_core();
        if ((abs(current_lat - target_lat) <= 0.000018) && \
            ((abs(current_long - target_long) <= 0.000018)))
        {
            publish_info("Within Target Bounds!");
            return true;
        }
        else
            return false;
    }

    bool check_macula_target()
    {
        publish_debug("Running Function: check_macula_target()");
        confirm_core();
        if ((abs(current_lat - macula_lat) <= 0.000018) && \
            ((abs(current_long - macula_long) <= 0.000018)))
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
        publish_debug("Running Function: set_bearing()");
        double X, Y, neededHeading;
        double deltaLong = target_long - current_long;
        double deg2rad = (3.131592/180);
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
        publish_debug("Running Function: set_distance_remaining()");
        double deg2rad = (180.0/3.141592);
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

    //-------------------------------------------------------------------------
    // AruCo Range finding
    //-------------------------------------------------------------------------
    // Finds range of AruCo detected 
    void range_aruco()
    {
        publish_info("Starting Function: range_aruco()");

        int midpoint, pog_checker;
        float pixelHeight, actualHeight, pixelWidth, actualWidth, distanceFromW = 0,
            range, lastRange;
        double x_offset, y_offset;
        double lat_offset, long_offset;
        double need_heading;
        // Focal Ratio is camera dependent.
        // For URC 2024 cam: 475.488
        // For IP cam: 
        // float focalRatio = 475.488;
        float theta;
        double deg2rad = (3.141592/180);
        double rad2deg = (180/3.141592);
        
        // refresh();
        midpoint = (abs(x0_c - x1_c));
        need_heading = current_heading + ((320 - midpoint) * -0.046875);


        pixelHeight = y3_c - y0_c;
        actualHeight = 0.15;
        pixelWidth = x1_c - x0_c;
        actualWidth = 0.15;
        distanceFromW = (FOCAL_RATIO/pixelWidth) * actualWidth;
        range = distanceFromW;
        float estAttempts = range/2.5;

        // get rid of weird stderr output
        (void)pog_checker;(void)pixelHeight;(void)actualHeight;(void)lastRange;
        (void)need_heading;(void)rad2deg;(void)estAttempts;
        
        theta = current_heading;
        if (theta > 90 && theta < 180)
        { 
            theta = 180 - theta;
        }
        else if (theta > 180 && theta < 270)
        {
            theta = theta - 180;
        }
        else if (theta > 270 && theta < 360)
        {
            theta = 360 - theta;
        }

        // Get offsets
        x_offset = range * abs(std::sin(theta * deg2rad));
        y_offset = range * abs(std::cos(theta * deg2rad));
        lat_offset = x_offset / 111139;
        long_offset = y_offset / 111139;


        // Fix circle
        if (imu_bearing > 180)
            x_offset *= -1;
        if (imu_bearing > 90 && imu_bearing < 270)
            y_offset*= -1;

        macula_lat = current_lat + lat_offset;
        macula_long = current_long + long_offset;
        macula_range = range;

        set_bearing();
    }

    //-------------------------------------------------------------------------
    // Camera Calibration
    //-------------------------------------------------------------------------
    // Using target_radius as range, this is used to find focal ratio
    void calibrate_camera()
    {
        publish_info("Starting Function: calibrate_camera()");

        int midpoint, pog_checker;
        float pixelHeight, actualHeight, pixelWidth, actualWidth, distanceFromW = 0,
            range, lastRange;
        double x_offset, y_offset;
        double lat_offset, long_offset;
        double need_heading;
        // Focal Ratio is camera dependent.
        // For URC 2024 cam: 475.488
        // For IP cam: 
        // float focalRatio = 475.488;
        float theta;
        double deg2rad = (3.141592/180);
        double rad2deg = (180/3.141592);
        
        // refresh();
        midpoint = (abs(x0_c - x1_c));
        need_heading = current_heading + ((320 - midpoint) * -0.046875);


        pixelHeight = y3_c - y0_c;
        actualHeight = 0.15;
        pixelWidth = x1_c - x0_c;
        actualWidth = 0.15;

        float focal = (target_radius/actualWidth) * pixelWidth;
        RCLCPP_INFO(this->get_logger(), "Find focal ratio: '%f'", focal);
    }
    
    //=======================================================================//
    //= ROS2 Shortcuts                                                      =//
    //=======================================================================//
    // Ways to make ROS2 easier to write

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
    rclcpp::Publisher<ros2_interfaces_pkg::msg::CoreControl>::SharedPtr publisher_core;
    rclcpp::Publisher<ros2_interfaces_pkg::msg::AutoNav>::SharedPtr publisher_nav;
    size_t count_;
    rclcpp_action::Server<NavigateRover>::SharedPtr navigate_rover_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
};

//====================================================================================
// Main
//====================================================================================
int main(int argc, char **argv)
{
    //Generates AruCo tags
    cv::Mat markerImage;
    cv::aruco::Dictionary dictionary1 = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::aruco::generateImageMarker(dictionary1, 1, 200, markerImage, 1);
    cv::imwrite("marker2.png", markerImage);

    //Camera stuff for OpenCV
    



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