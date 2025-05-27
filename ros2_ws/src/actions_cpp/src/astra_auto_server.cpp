//=============================================================================
//rover-Autonomy Server
//runs commands from the client
//Last edited May 20, 2025
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

//Made by Daegan Brown for ASTRA
#include "pathfind.h"                       // My functions

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


// Checks
bool coreWait = 1;




//=============================================================================
//ROS2 Listener Nodes
//=============================================================================


// Node for subcribing to topics
class NavigateRoverSubscriberNode : public rclcpp::Node 
{
public:
    
    NavigateRoverSubscriberNode() : Node("navigate_rover_subscriber")
    {
        subscriber_core_ = this->create_subscription<ros2_interfaces_pkg::msg::CoreFeedback>(
            "/core/feedback", 10, std::bind(&NavigateRoverSubscriberNode::topic_callback, this, _1));

    }
private:
    //=========================================================================
    // Subscriber Topic Callback
    //=========================================================================
    void topic_callback(const ros2_interfaces_pkg::msg::CoreFeedback & msg) 
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

    rclcpp::Subscription<ros2_interfaces_pkg::msg::CoreFeedback>::SharedPtr subscriber_core_;

};

//=============================================================================
// ROS2 Server Node
//=============================================================================
// Node for the action server
class NavigateRoverServerNode : public rclcpp::Node 
{
public:
    //=========================================================================
    // Constructor
    //=========================================================================
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

        //Creating Publisher that communicates to the motors
        publisher_motors = this->create_publisher<std_msgs::msg::String>(
            "astra/core/control", 10);
        
        //Creating a publisher that asks for imu data
        publisher_feedback = this->create_publisher<std_msgs::msg::String>(
            "astra/auto/feedback", 10);

        // Publisher to send information directly to anchor
        publisher_anchor = this->create_publisher<std_msgs::msg::String>(
            "/anchor/relay", 10);
        
    }
    
private:
    //=========================================================================
    // Internal Variables
    //=========================================================================
    // Mission Info
    int mission_type;
    double target_lat;
    double target_long;
    double target_radius;
    double period;

    //=========================================================================
    // Callback of recieved Goal
    //=========================================================================
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
    //=========================================================================
    // Cancelling the Goal
    //=========================================================================
    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<NavigateRoverGoalHandle> goal_handle)
    {
        // Get rid of stdr output at build
        (void)goal_handle;
        std::string rover_command;
        auto message_motors = std_msgs::msg::String();

        rover_command = "ctrl,0,0";  
        message_motors.data = rover_command;
        

        publisher_motors->publish(message_motors);

        //FEEDBACK
        RCLCPP_INFO(this->get_logger(), "Recieved Goal Cancel Request but WONT FUCKIN DO IT");
        cancel_request = true;
        
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    //=========================================================================
    // Once Accepted
    //=========================================================================
    void handle_accepted_callback(
        const std::shared_ptr<NavigateRoverGoalHandle> goal_handle)
    {
        
        RCLCPP_INFO(this->get_logger(), "Executing the goal");
        legacy_execute_goal(goal_handle);
    }
    //=========================================================================
    // Execution of the Goal
    //=========================================================================
    void execute_goal(
        const std::shared_ptr<NavigateRoverGoalHandle> goal_handle)
    {
        // Set microsecond values for usleep command
        unsigned int u_second = 1000000;
        auto result = std::make_shared<NavigateRover::Result>();

        // Get action goal data
        mission_type = goal_handle->get_goal()->mission_type;
        target_lat = goal_handle->get_goal()->gps_lat_target;
        target_long = goal_handle->get_goal()->gps_long_target;
        target_radius = goal_handle->get_goal()->target_radius;
        period = goal_handle->get_goal()->period;

        publish_info("Recieved New Goal");
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
                    usleep(4 * u_second);
                    set_led(2);
                    usleep(4 * u_second);
                    set_led(3);
                    usleep(4 * u_second);
                    set_led(1);
                    // Face target
                    float target_bearing = (float)find_facing(target_lat, target_long,
                        current_lat, current_long);
                    orient(target_bearing);
                        result->final_result = 0;
                }
                break;
            //-----------------------------------------------------------------
            // Case 1:
            // Rover will point to target, drive forward 3 seconds, then 
            // repeat until within target radius or goal is canceled.
            //-----------------------------------------------------------------
            case 1:
                while (!(checkTarget()) && !(goal_handle->is_canceling()))
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
            usleep(0.5 * u_second);
            set_led(0);
            usleep(0.5 * u_second);
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

        // GNSS Goal
        else if (navigate_type == 1)
        {
            //FEEDBACK
            message_feedback.data = "Starting navigation to GNSS point";
            publisher_feedback->publish(message_feedback);

            //Make Rover face North, announce chosen task,
            message_motors.data = "auto,turningTo,15000,0";
            publisher_motors->publish(message_motors);
            std::cout << "Selected GPS targeting" << std::endl;

            //This loop gets the rover to continually get closer to the GPS 
            //target until it is within a half meter. 
            while (iterate == 0) // REMOVE THIS SHIT, make it  || not_canceld, essentially
            {
                
                
                message_motors.data = "data,getOrientation";
                publisher_motors->publish(message_motors);
                usleep(0.5 * microsecond);
                
                

                message_motors.data = "data,sendGPS";
                publisher_motors->publish(message_motors);
                usleep(0.5 * microsecond);
                
                current_lat = imu_command_gps(gps_string,1);
                current_long = imu_command_gps(gps_string,2);

                needDistance = find_distance(gps_lat_target, gps_long_target, current_lat, current_long);
                i_needDistance = needDistance;
                RCLCPP_INFO(this->get_logger(), "Remaining distance: '%d'", i_needDistance);


                i_needHeading = find_facing(gps_lat_target, gps_long_target, current_lat, current_long);
                
                
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

                if ((abs(current_lat - gps_lat_target) <= 0.000018) && \
                    ((abs(current_long - gps_long_target) <= 0.000018) ))
                {
                    
                    iterate++;
                    //FEEDBACK
                    message_feedback.data = "Arrived at point";
                    publisher_feedback->publish(message_feedback);
                }
                
            }
             
            message_motors.data = "led_set,0,0,300";
            publisher_motors->publish(message_motors);
            RCLCPP_INFO(this->get_logger(), "Arrived at location"); 
            std::cout << "Arrived at location!";           
            
        }

        // Aruco Goal
        else if (navigate_type == 2)
        {
            
        }

        // Object Detection Goal
        else if (navigate_type == 3)
        {

        }


        //*****************************************************************************************
        // Debug Goals
        //*****************************************************************************************


        // DEBUG 1
        //This is a test loop. Same code as 1, the only difference is instead of drving until it 
        //reaches the point, it faces it once, drives towards it once, then is done. 
        else if (navigate_type == 4)
        {
            //FEEDBACK
            message_feedback.data = "Selected DEBUG 1: Non iterating GNSS Navigation";
            publisher_feedback->publish(message_feedback);
            std::cout << "Selected GPS targeting, but once" << std::endl;
            while (iterate == 0)
            {
                message_motors.data = "auto,turningTo,15000,0";
                
                publisher_motors->publish(message_motors);
                //usleep(15 * microsecond);
                
                
                
                message_motors.data = "data,getOrientation";
                publisher_motors->publish(message_motors);
                usleep(3 * microsecond);
                for (int i; i<5000; i++)
                {
                    usleep(1000);
                }

                message_motors.data = "data,sendGPS";
                publisher_motors->publish(message_motors);
                //usleep(3 * microsecond);
                for (int i; i<5000; i++)
                {
                    usleep(1000);
                }
                std::cout << gps_string << std::endl;  
                current_lat = imu_command_gps(gps_string,1);
                current_long = imu_command_gps(gps_string,2);


                i_needHeading = find_facing(gps_lat_target, gps_long_target, \
                    current_lat, current_long);
                
                
                std::cout << std::fixed << "Calculated Heading: " << i_needHeading << std::endl \
                    << std::endl << std::endl << std::endl;


                message_motors.data = "auto,turningTo,15000," + std::to_string(i_needHeading);
                publisher_motors->publish(message_motors);
                usleep(15 * microsecond);

                rover_command = "ctrl,0,0";  
                message_motors.data = rover_command;
                RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
                publisher_motors->publish(message_motors);

                message_motors.data = "data,getGPS";
                publisher_motors->publish(message_motors);
                usleep(100000);
                current_lat = imu_command_gps(gps_string,1);
                current_long = imu_command_gps(gps_string,2);
                    
                iterate++;
                //FEEDBACK
                message_feedback.data = "Finished DEBUG 1";
                publisher_feedback->publish(message_feedback);
                
            }
        }

        // DEBUG 2
        else if (navigate_type == 5)
        {
            //FEEDBACK
            message_feedback.data = "Selected DEBUG 2: Test motors and IMU";
            publisher_feedback->publish(message_feedback);


            rover_command = "ctrl,-.50,-.50";
            message_motors.data = rover_command;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
            publisher_motors->publish(message_motors);
            usleep(3.0 * microsecond);

            rover_command = "ctrl,0.0,0.0";
            message_motors.data = rover_command;
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_motors.data.c_str());
            publisher_motors->publish(message_motors);
            usleep(1.5 * microsecond);


            // rover_command = "auto,turningTo,15000,180";
            // message_motors.data = rover_command;
            // publisher_motors->publish(message_motors);
            // usleep(15.0 * microsecond);

            //FEEDBACK
            message_feedback.data = "Finished DEBUG 2: Testing";
            publisher_feedback->publish(message_feedback);
        }
        
        // DEBUG 3 
        //Rectilinear search pattern, mostly for SAR filming
        else if (navigate_type == 6)
        {
            //FEEDBACK
            message_feedback.data = "Selected DEBUG 3: Test Search Pattern";
            publisher_feedback->publish(message_feedback);


            std::cout << "Selected Square Search Pattern" << std::endl;
            message_motors.data = "led_set,303,0,0";
            publisher_motors->publish(message_motors);
            RCLCPP_INFO(this->get_logger(), "Begining Search");
            //
            for (int j = 0; j < gps_lat_target; j++)
            {
                //Rover faces North
                message_motors.data = "auto,turningTo,15000,350";
                publisher_motors->publish(message_motors);
                usleep(5 * microsecond);

                //Begins driving forward at 40% speed
                message_motors.data = "ctrl,-0.4,-0.4";
                publisher_motors->publish(message_motors);
                //Rover waits for the third input seconds before stopping
                usleep(gps_long_target * microsecond);
                message_motors.data = "ctrl,0.0,0.0";
                publisher_motors->publish(message_motors);
                //Rover faces East
                message_motors.data = "auto,turningTo,15000,80";
                publisher_motors->publish(message_motors);
                usleep(5 * microsecond);

                //Rover moves forward for a second before stopping againt
                message_motors.data = "ctrl,-0.4,-0.4";
                publisher_motors->publish(message_motors);
                usleep(4 * microsecond);
                message_motors.data = "ctrl,0.0,0.0";
                publisher_motors->publish(message_motors);
                //Rover faces South and drives that way for the same amount of time before stopping
                message_motors.data = "auto,turningTo,15000,170";
                publisher_motors->publish(message_motors);
                usleep(5 * microsecond);
                message_motors.data = "ctrl,-0.4,-0.4";
                publisher_motors->publish(message_motors);
                usleep(gps_long_target * microsecond);
                message_motors.data = "ctrl,0.0,0.0";
                publisher_motors->publish(message_motors);
                //Faces East again, moves forward one second, and stops the loop
                message_motors.data = "auto,turningTo,15000,80";
                publisher_motors->publish(message_motors);
                usleep(5 * microsecond);
                message_motors.data = "ctrl,-0.4,-0.4";
                publisher_motors->publish(message_motors);
                usleep(4 * microsecond);
                message_motors.data = "ctrl,0.0,0.0";
                publisher_motors->publish(message_motors);
            }
            //FEEDBACK
            message_feedback.data = "Finished DEBUG 3: Area Searched";
            publisher_feedback->publish(message_feedback);
        }
        
        // DEBUG 4
        else if (navigate_type == 7)
        {
            //FEEDBACK
            message_feedback.data = "Selected DEBUG 4: ARUCO Test";
            publisher_feedback->publish(message_feedback);

            std::cout << "Looking For ARCUO" << std::endl;
            int cameraNum;
            std::cin >> cameraNum;
            cv::VideoCapture inputVideo(cameraNum);
            
            
            //inputVideo.open(cameraNum);
            cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
            cv::aruco::Dictionary dictionary = \
                cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
            cv::aruco::ArucoDetector detector(dictionary, detectorParams);



            //cv::Size S = cv::Size((int) inputVideo.get(cv::CAP_PROP_FRAME_WIDTH), // Acquire input size
              //  (int) inputVideo.get(cv::CAP_PROP_FRAME_HEIGHT));

            //cv::VideoWriter writer;
            //int codec = cv::VideoWriter::fourcc('a', 'v', 'c', '1');
            //double fps = 15.0;
            //std::string filename = "footage.mp4";
            //cv::Size sizeFrame(640,480);
            //writer.open(filename, codec, fps, sizeFrame, true);
            //cv::Mat cameraMatrix, distCoeffs;
            //float markerLength = 0.05;

            //readCameraParameters(cameraParamsFilename, cameraMatrix, distCoeffs);

            //cv::Mat objPoints(4, 1, CV_32FC3);
            //objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
            //objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
            //objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
            //objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);

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
            while (inputVideo.grab()) 
                {
                iterateIT ++;
                cv::Mat image, imageCopy;
                inputVideo.retrieve(image);
                image.copyTo(imageCopy);
                //std::vector<int> ids;
                //std::vector<std::vector<cv::Point2f>> corners, rejected;
                detector.detectMarkers(image, corners, ids, rejected);
                // if at least one marker detected
                if (ids.size() > 0)
                    cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);

                //int nMarkers = corner.size();
                //std::vector<cv::Vec3d> rvecs(nMarkers), tvecs(nMarkers);

                //Calculate pose for each marker
                //for (int i = 0; i < nMarkers; i++)
                //{
                //solvePnP(objPoints, corner.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
                //}

                //Draw Axis for each marker
                //for (unsigned int i = 0; i < ids.size(); i++)
                //{
                //    cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1)
                //}

                outputVideo.write(imageCopy);




                cv::imshow("out", imageCopy);
                //inputVideo >> imageCopy;
                //cv::Mat xframe;
                //resize(imageCopy, xframe, sizeFrame);
                //writer.write(xframe);


                //split(src, spl);                // process - extract only the correct channel
                //for (int i =0; i < 3; ++i)
                //{
                    
                //spl[i] = cv::Mat::zeros(S, spl[0].type());
                //    
                //}
                //merge(spl, res);

               // outputVideo.write(imageCopy);
                //outputVideo.write(res); //save or
                //outputVideo << res;

                //int waitTime = 1;
                char key = (char) cv::waitKey(1);
                if (key == 27)
                //std::cout << iterateIT << std::endl;
                //if (iterateIT >= 90)
                {
                    break;
                }
                
                

                }
               

            std::cout << "Finished filming!" << std::endl;
            inputVideo.release();
            //writer.release();
        }   

        // DEBUG 5
        else if (navigate_type == 8)
        {
            //FEEDBACK
            message_feedback.data = "Selected DEBUG 5: ??????????";
            publisher_feedback->publish(message_feedback);

             // Get the message describing the goal
            auto feedback_msg = std::make_shared<NavigateRover::Feedback>();
            // 
            auto &status_message = feedback_msg->current_status;

            status_message = 99;


            // Send some data back to the goal
            for (int i = 0; i < 2; i++) goal_handle->publish_feedback(feedback_msg);

            usleep(5000);
            if (rclcpp::ok()) {
            // Publish the result
            auto result = std::make_shared<NavigateRover::Result>();
            result->final_result = 10;

            
            }
        }

        // DEBUG 6
        else if (navigate_type == 9)
        {
            //FEEDBACK
            message_feedback.data = "Selected DEBUG 6: Counting to 500";
            publisher_feedback->publish(message_feedback);

            for (int i = 0; i <= 500; i++)
            {
                if (goal_handle->is_canceling())
                {
                    goal_handle->canceled(0);
                    return;
                }
                std::cout << i << std::endl;
                usleep(.1 * microsecond);
            }
        }   

        //*****************************************************************************************
        // Internal Goals
        //*****************************************************************************************

        // INTERNAL 1
        else if (navigate_type == 10)
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

    //Subscriber to astra/core/feedback
    
    //=========================================================================
    // DEBUG This shouldn't be here...
    //=========================================================================
    void topic_callback(const std_msgs::msg::String & msg) 
    {
        std::string command;
        command = msg.data;
        RCLCPP_INFO(this->get_logger(), "Recieved: '%s'", msg.data.c_str());
        

        
        std::string delimiter = ",";
            size_t pos = 0;
            std::string token;
            std::string token1;
            std::string scommand = command.c_str();
            pos = scommand.find(delimiter);
            token = scommand.substr(0, pos);
            

        if (token == "orientation")
        {
            RCLCPP_INFO(this->get_logger(), "Recieved IMU bearing");
            //Turns command into the proper bearing
            //imu_bearing = imu_command(command); 
        }
        else if (token == "gps")
        {
            RCLCPP_INFO(this->get_logger(), "Recieved GPS location");
            //Turns command into GPS string
            gps_string = command;
        }

    }

    //=========================================================================
    // Set motors
    //=========================================================================

    void set_motors(int state)
    {
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

    //=========================================================================
    // Orient To
    //=========================================================================
    
    void orient(float bearing)
    {
        auto message = ros2_interfaces_pkg::msg::CoreControl();
        message.turn_to_enable = false;
        message.turn_to = bearing;
        message.turn_to_timeout = 10;
        std::string msg = "Turning to face " + std::to_string(bearing); 
        const char * c_msg = msg.c_str();
        publish_info(c_msg);

        publisher_core->publish(message);
        
    }

    //=========================================================================
    // Head to Point
    //=========================================================================

    void head_to_point()
    {

    }

    //=========================================================================
    // Legacy AruCo Code
    //=========================================================================

    void legacy_aruco()
    {

    }

    //=========================================================================
    // Legacy Search
    //=========================================================================

    void legacy_search()
    {

    }
    
    //=========================================================================
    // Set LED
    //=========================================================================
    // 0 = OFF
    // 1 = RED
    // 2 = GREEN
    // 3 = BLUE
    void set_led(int color)
    {
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

    bool checkTarget()
    {
        coreWait = 1;
        publish_debug("Waiting for core feedback before checking target");
        while (coreWait);
        if ((abs(current_lat - target_lat) <= 0.000018) && \
            ((abs(current_long - target_long) <= 0.000018)))
        {
            publish_info("Within Target Bounds!");
            return true;
        }
        else
            return false;
    }

    //=========================================================================
    // Publish Debug, Info, Warn, Error, Fatal
    //=========================================================================
    
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


    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_motors;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_feedback;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_anchor;
    //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Publisher<ros2_interfaces_pkg::msg::CoreControl>::SharedPtr publisher_core;
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