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

// ROS2 Comms
#include "ros2_interfaces_pkg/action/auto_command.hpp"
#include "ros2_interfaces_pkg/msg/core_feedback.hpp"
#include "ros2_interfaces_pkg/msg/core_control.hpp"
#include "ros2_interfaces_pkg/msg/auto_feedback.hpp"
#include "ros2_interfaces_pkg/msg/auto_nav.hpp" 
// #include "ros2_interfaces_pkg/msg/"

//============
// Definitions
//============

#define SECOND 1000000
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


// Flags
bool canceled = 0;
bool coreWait = 1;                  // Is it waiting on /core/feedback?
bool anchorWait = 0;                // Is it waiting on /anchor/core/feedback?
bool arucoFound = 0;
bool objectFound = 0;
bool navFail = 0;




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
        subscriber_macula_ = this->create_subscription<ros2_interfaces_pkg::msg::Macula
        
        

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
        if (anchorWait == 0)
            return;
        if (msg.data == "")
        {
            anchorWait = 0;
        }
    }
    rclcpp::Subscription<ros2_interfaces_pkg::msg::CoreFeedback>::SharedPtr subscriber_core_;
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
        if (goal->mission_type > 15 || goal->mission_type < 0)
        {
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

                }
                break;
        }
        if (goal_handle->is_canceling())
        {
            publish_info("Goal was canceled!");
            publish_debug("Cancel happened during switch.");
            set_led(0);
        }


        // Send Result

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

    //=========================================================================
    // Legacy Execution of Goal
    //=========================================================================
    void legacy_execute_goal(
        const std::shared_ptr<NavigateRoverGoalHandle> goal_handle)
    {
        // Set microsecond values for usleep command
        // DEBUG change variable names
        unsigned int microsecond = 1000000;
        auto result = std::make_shared<NavigateRover::Result>();

        // Get Request from goal
        int command_1 = goal_handle->get_goal()->mission_type;
        double command_2 = goal_handle->get_goal()->gps_lat_target;
        double command_3 = goal_handle->get_goal()->gps_long_target;
        double command_4 = goal_handle->get_goal()->target_radius;
        double command_5 = goal_handle->get_goal()->period;

        int navigate_type = command_1;
        double gps_lat_target = command_2;
        double gps_long_target = command_3;
        double target_radius = command_4; (void)target_radius;
        double period = command_5;

        // Execute the action
        int final_result = 0;
        std::string rover_command;
        rclcpp::Rate loop_rate(1.0/period);
        
        // Switch Statement determining what type of action is being asked of the 
        // rover. 
        // 0: Stops rover
        // 1: Simply go to GPS coordinates, stop, and signal.
        // 2: Go and search target area for aruco tags
        // 3: Go and search target area for objects
        // 4: 1 but only looping once
        // 5: Goes forward. Used for testing. 
        // 6: Search pattern
        // 7: AruCo Test
        // 8: Object Detection

        // 10: ARUCO detected Message
        // 11: Object detected Message
        auto message_motors = std_msgs::msg::String();
        auto message_feedback = std_msgs::msg::String();
        
        double current_lat;
        double current_long;
        //double bearing;
        //float currentHeading;
        //float needHeading = 0;
        double needDistance;
        int i_needDistance;
        int i_needHeading;
        int iterate = 0;


        //FEEDBACK
        message_feedback.data = "Autonomy starting up. Cycling lights.";
        publisher_feedback->publish(message_feedback);


        //Turn LEDs red 
        message_motors.data = "led_set,300,0,0";
        publisher_motors->publish(message_motors);


        //Request GPS data from Core, then wait 3 seconds.
        message_motors.data = "data,sendGPS";
        publisher_motors->publish(message_motors);
        usleep(3 * microsecond);

        //Use first input to decide where to go. Switch statement could work
        //better, TBD

        //=====================================================================
        // Core Goals
        //=====================================================================
        
        //Check Cancel
        if (goal_handle->is_canceling())
        {
            //FEEDBACK
            message_feedback.data = "Ending goal";
            publisher_feedback->publish(message_feedback);
            result->final_result = 1;
            goal_handle->canceled(result);
            
            return;
        }

        // Stop Goal
        if (navigate_type == 0)
        {
            //FEEDBACK
            message_feedback.data = "Selected STOP";
            publisher_feedback->publish(message_feedback);

            message_motors.data = "ctrl,0,0";
            RCLCPP_INFO(this->get_logger(), "Stopping");
            publisher_motors->publish(message_motors);
        }

        
        // INTERNAL 1
        if (navigate_type == 10)
        {
            //FEEDBACK
            message_feedback.data = "Aruco Tag detected! Homing in";
            publisher_feedback->publish(message_feedback);

            int x_coord = command_2;
            int x2_coord = command_3;
            int x3_coord = 0; (void)x3_coord;
            int x4_coord = 0; (void)x4_coord;
            int y_coord = 0; 
            int y2_coord = 0; (void)y2_coord;
            int y3_coord = 0; (void)y3_coord;
            int y4_coord = 0;
            int pog_checker = 0; (void)pog_checker;
            int midpoint = 0;
            float pixelHeight; (void)pixelHeight;
            float actualHeight; (void)actualHeight;
            float pixelWidth ;
            float actualWidth;
            float distanceFromW = command_4;
            float range;
            float lastRange; (void)lastRange;
            double x_offset, y_offset;
            double lat_offset, long_offset;
            //CHANGE PER CAMERA
            //MAY NEED CALIBRATING
            float focalRatio = 475.488;
            float theta;
            bool found = false;
            bool firstFrame = false;



            double deg2rad = (3.141592/180);
            double rad2deg = (180/3.141592); (void)rad2deg;

            std::cout << "Homing in on Aruco" << std::endl;
            // int cameraNum = 10;
            //std::cin >> cameraNum;
            cv::VideoCapture inputVideo("/dev/video10");
            cv::Mat camMatrix, distCoeffs;
            
            
            //inputVideo.open(cameraNum);
            cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
            cv::aruco::Dictionary dictionary = \
                cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            cv::aruco::ArucoDetector detector(dictionary, detectorParams);
            
            cv::Mat image, imageCopy;
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners, rejected;
            inputVideo >> image;
            std::cout << "Video Prepared" << std::endl;

            cv::Mat res;
            std::vector<cv::Mat> spl;
            cv::VideoWriter outputVideo;    
            // select desired codec (must be available at runtime)
            int codec = cv::VideoWriter::fourcc('H', '2', '6', '4');  
            double fps = 25.0;                          // framerate of the created video stream
            std::string filename = "./live.mp4";             // name of the output video file
            outputVideo.open(filename, codec, fps, image.size(), true);
            // check if we succeeded
            if (!outputVideo.isOpened()) {
                std::cerr << "Could not open the output video file for write\n";
                
                }



                std::cout << "Output prepared" << std::endl;
            int iterateIT = 0;
            int estAttempts;
            
            
            while (inputVideo.grab()) 
            {
                iterateIT ++;
                // std::cout << "Attempt " << iterateIT << std::endl;
                cv::Mat image, imageCopy;
                inputVideo.retrieve(image);
                
                cv::resize(image, imageCopy, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
                //cv::namedWindow("out", CV_WINDOW_AUTOSIZE);
                //std::vector<int> ids;
                //std::vector<std::vector<cv::Point2f>> corners, rejected;
                detector.detectMarkers(imageCopy, corners, ids, rejected);
                // if at least one marker detected
                // int debug_iterator = 0;
                if (ids.size() > 0)
                {
                    /*
                    int Xdebug_aruco = (int)rejected[0][0].x;
                    int Ydebug_aruco = (int)rejected[0][0].y;
                    std::cout << '{' << Xdebug_aruco << ',' << Ydebug_aruco << '}' << std::endl;
                    int Xids = (int)ids[0];
                    std::cout << Xids << std::endl;
                    */
                    //FEEDBACK
                    message_feedback.data = "Aruco Tag detected! Homing in";
                    publisher_feedback->publish(message_feedback);
                    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
                    std::cout << "Aruco Detected" << std::endl;
                    x_coord = (int)corners[0][0].x;
                    y_coord = (int)corners[0][0].y;

                    x2_coord = (int)corners[0][1].x;
                    y2_coord = (int)corners[0][1].y;

                    x3_coord = (int)corners[0][2].x;
                    y3_coord = (int)corners[0][2].y;

                    x4_coord = (int)corners[0][3].x;
                    y4_coord = (int)corners[0][3].y;
                    found = true;
                    firstFrame = true;

      
                }
                estAttempts = distanceFromW/2.5;
                 
                outputVideo.write(imageCopy);

                message_motors.data = "data,getOrientation";
                publisher_motors->publish(message_motors);
                usleep(1 * microsecond);
                // imu_bearing = orientation_string(command);

                //*********************************************************************************
                // Face Tag
                //*********************************************************************************
                if (found)
                {
                    midpoint = (abs(x_coord - x2_coord));
                    i_needHeading = imu_bearing + ((320 - midpoint) * -0.046875);
                    // if (abs(320 - midpoint) <= 5)
                    // {
                    //     //You chill
                    //     //FEEDBACK
                    //     message_feedback.data = "Perfect Heading";
                    //     publisher_feedback->publish(message_feedback);
                    // }
                    // else if (abs(320-midpoint) <= 30)
                    // {
                    //     if (midpoint < 320)
                    //         i_needHeading = imu_bearing - 3;
                    //     else
                    //         i_needHeading = imu_bearing + 3;

                    //     if (imu_bearing < 0)
                    //         imu_bearing = imu_bearing + 360;
                    //     else if (imu_bearing > 360)
                    //         imu_bearing = imu_bearing - 360;
                    //     //FEEDBACK
                    //     message_feedback.data = "Good Heading";
                    //     publisher_feedback->publish(message_feedback);
                    //     message_motors.data = "auto,turningTo,15000," + std::to_string(i_needHeading);
                    //     publisher_motors->publish(message_motors);
                    // }
                    // else if (abs(320-midpoitn) <= 100)
                    // {
                    //     if (midpoint < 320)
                    //         i_needHeading = imu_bearing - 5;
                    //     else
                    //         i_needHeading = imu_bearing + 5;

                    //     if (imu_bearing < 0)
                    //         imu_bearing = imu_bearing + 360;
                    //     else if (imu_bearing > 360)
                    //         imu_bearing = imu_bearing - 360;
                        
                    //     //FEEDBACK
                    //     message_feedback.data = "Mediocre Heading";
                    //     publisher_feedback->publish(message_feedback);
                    // }
                    // else if (abs(320-midpoint) <= 200)
                    // {
                    //     if (midpoint < 320)
                    //         i_needHeading = imu_bearing - 10;
                    //     else
                    //         i_needHeading = imu_bearing + 10;

                    //     if (imu_bearing < 0)
                    //         imu_bearing = imu_bearing + 360;
                    //     else if (imu_bearing > 360)
                    //         imu_bearing = imu_bearing - 360;
                    //     //FEEDBACK
                    //     message_feedback.data = "Poor Heading";
                    //     publisher_feedback->publish(message_feedback);
                    // }


                    message_motors.data = "data,getOrientation";
                    publisher_motors->publish(message_motors);
                    usleep(0.5 * microsecond);

                    //*********************************************************************************
                    // Calculate Distance
                    //*********************************************************************************

                    pixelHeight = y4_coord - y_coord;
                    actualHeight = .15;
                    pixelWidth = x2_coord - x_coord;
                    actualWidth = .15;
                    distanceFromW = (focalRatio/pixelWidth) * actualWidth;
                    range = distanceFromW;
                    //FEEDBACK
                    message_feedback.data = range;
                    publisher_feedback->publish(message_feedback);
                    
                    estAttempts = range/2.5;
                    

                    //FEEDBACK
                    message_feedback.data = ("ARUCO detected at range of '%s' meters", message_feedback.data.c_str());
                    publisher_feedback->publish(message_feedback);

                    theta = imu_bearing;
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

                    x_offset = range * abs(std::sin(theta * deg2rad));
                    y_offset = range * abs(std::cos(theta * deg2rad));

                    lat_offset = x_offset / 111139;
                    long_offset = y_offset / 111139;

                    if (imu_bearing > 180)
                    {
                        x_offset = x_offset * -1;
                    }
                    if (imu_bearing > 90 && imu_bearing < 270)
                    {
                        y_offset = y_offset * -1;
                    }
                    

                    message_motors.data = "data,sendGPS";
                    publisher_motors->publish(message_motors);
                    usleep(0.75 * microsecond);
                    
                    current_lat = imu_command_gps(gps_string,1);
                    current_long = imu_command_gps(gps_string,2);

                    
                    gps_lat_target = current_lat + lat_offset;
                    gps_long_target = current_long + long_offset;
                }

                // i_needHeading = find_facing(gps_lat_target, gps_long_target, current_lat, current_long);
                
                
                std::cout << std::fixed << "Calculated Heading: " << i_needHeading << std::endl \
                    << std::endl << std::endl << std::endl;


                message_motors.data = "auto,turningTo,15000," + std::to_string(i_needHeading);
                publisher_motors->publish(message_motors);
                usleep(3.5 * microsecond);

                
                rover_command = "ctrl,-0.6,-0.6";  
                message_motors.data = rover_command;
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
                publisher_motors->publish(message_motors);
                usleep(1.5 * microsecond);
                

                rover_command = "ctrl,0,0";  
                message_motors.data = rover_command;
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
                publisher_motors->publish(message_motors);

                message_motors.data = "data,getGPS";
                publisher_motors->publish(message_motors);
                usleep(100000);
                current_lat = imu_command_gps(gps_string,1);
                current_long = imu_command_gps(gps_string,2);

                if ((abs(current_lat - gps_lat_target) <= 0.00002) && \
                    ((abs(current_long - gps_long_target) <= 0.00002) ))
                {
                    estAttempts = 0;
                    //FEEDBACK
                    message_feedback.data = "Arrived at point";
                    publisher_feedback->publish(message_feedback);
                }


                


                found = false;
                estAttempts = estAttempts - 1;
                if (firstFrame && estAttempts == 0)
                    break;
                //End the Loop

            }
            //Close video Stream
            std::cout << "Finished filming!" << std::endl;
            inputVideo.release();
            
            
    
            //FEEDBACK
            message_feedback.data = "ARUCO Found succesfully";
            publisher_feedback->publish(message_feedback);
               
            message_motors.data = "led_set,0,0,300";
            publisher_motors->publish(message_motors);
            std::cout << "Target Found!" << std::endl;
            inputVideo.release();
        }

        // INTERNAL 2
        else if (navigate_type == 11)
        {
            //FEEDBACK
            message_feedback.data = "Object detected! Homing in";
            publisher_feedback->publish(message_feedback);

            message_feedback.data = "me when I lie, we ain't finding it";
            publisher_feedback->publish(message_feedback);
        }
        

        // Pause before stopping 


        usleep(3 * microsecond);
        message_motors.data = "ctrl,0,0";
        RCLCPP_INFO(this->get_logger(), "Stopping");
        publisher_motors->publish(message_motors);

        //FEEDBACK
        message_feedback.data = "Goal Finished";
        publisher_feedback->publish(message_feedback);

        
        


        // Set final state and return result
        
        result->final_result = final_result;
        goal_handle->succeed(result);
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
    //= Internal Calculations                                               =//
    //=======================================================================//
    // Internal calculations and such. 

    //-------------------------------------------------------------------------
    // Aruco Homing
    // This function is called when an AruCo tag is detected, to calculate the
    // distance to it 
    //-------------------------------------------------------------------------
    float aruco_homing()
    {

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