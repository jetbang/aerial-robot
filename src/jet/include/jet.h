/**
 * Copyright (c) 2017, Jack Mo (mobangjack@foxmail.com).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <ros/ros.h>
#include<tf/transform_broadcaster.h>

#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/server/simple_action_server.h>

#include <dji_sdk/dji_drone.h>
#include <vector>
#include <iostream> 
#include <iomanip>  

#include "pid.h"
#include "uart.h"
#include "crc8.h"
#include "timer.h"
#include "kbhit.h"

#include "jet/Charge.h"
#include "jet/CmdGrabber.h"
#include "jet/StatGrabber.h"
#include "jet/JetNavAction.h"

using namespace DJI::onboardSDK;

class Jet
{
public:
    typedef enum
    {
        STAND_BY,
        GRAB_BULLETS,
        REQUEST_CONTROL,
        TAKE_OFF,
        TO_NORMAL_ALTITUDE,
        FLY_TO_CAR,
        FIND_CAR,
        SERVE_CAR,
        DROP_BULLETS,
        BACK_TO_NORMAL_ALTITUDE,
        FLY_BACK,
        FIND_PARK,
        VISUAL_SERVO_LANDING,
        LANDING,
        RELEASE_CONTROL,
    } JetCmd_e;

public:
    Jet(ros::NodeHandle& nh);
    ~Jet();

protected:
    int uart_fd;
    std::string serial_port;
    int serial_baudrate;

    int spin_rate;

    std::vector<int> duration;

    ros::NodeHandle nh;

    ros::Subscriber odometry_sub;
    ros::Subscriber vision_sub;

    ros::Publisher jet_state_pub;
    ros::Publisher pose_calied_pub;

    ros::ServiceServer charge_srv;
    ros::ServiceServer cmd_grabber_srv;
    ros::ServiceServer stat_grabber_srv;
    ros::ServiceServer reload_pid_param_srv;
    ros::ServiceServer reload_vision_param_srv;
    ros::ServiceServer reload_flight_param_srv;
    ros::ServiceServer reload_dropoint_param_srv;
    ros::ServiceServer reload_duration_param_srv;

    typedef actionlib::SimpleActionServer<jet::JetNavAction> JetNavActionServer;
    JetNavActionServer* jet_nav_action_server;
    jet::JetNavFeedback jet_nav_feedback;
    jet::JetNavResult jet_nav_result;

    geometry_msgs::Vector3 dropoint;

protected:
    void calc_jet_pos_calied();
    void pub_pose_calied();

    void pub_jet_state();

    void load_dropoint_param(ros::NodeHandle& nh);
    void fill_pid_param(ros::NodeHandle& nh, int i, const char* axis);
    void load_pid_param(ros::NodeHandle& nh);
    void load_vision_param(ros::NodeHandle& nh);
    void load_flight_param(ros::NodeHandle& nh);
    void load_duration_param(ros::NodeHandle& nh);
    void load_timeout_param(ros::NodeHandle& nh);

protected:
    // subscriber callbacks
    void odometry_callback(const nav_msgs::OdometryConstPtr& odometry);
    void vision_callback(const geometry_msgs::PoseStamped& pose_stamped);

    // service callbacks
    bool charge_callback(jet::Charge::Request& request, jet::Charge::Response& response);
    bool cmd_grabber_callback(jet::CmdGrabber::Request& request, jet::CmdGrabber::Response& response);
    bool stat_grabber_callback(jet::StatGrabber::Request& request, jet::StatGrabber::Response& response);
    bool reload_pid_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reload_vision_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reload_flight_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reload_dropoint_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reload_duration_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reload_timeout_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    // action callbacks
    bool jet_nav_action_callback(const jet::JetNavGoalConstPtr& goal);

protected:
    DJIDrone drone;
    uint8_t jet_state;

    int odom_callback_timeout;
    int vision_callback_timeout;

    Timer odom_callback_timer;
    Timer vision_callback_timer;

    bool calied;

    float jet_pos_raw[4];
    float jet_pos_bias[4];
    float jet_pos_calied[4];

    geometry_msgs::PoseStamped pose_calied;

    float vision_target_local_pos_raw[4];
    float vision_target_global_pos_raw[4];

    int vision_target_pos_filter_window_size;
    double vision_target_pos_filter_variance_limit;
    std::vector<float> vision_target_local_pos_vec[4]; // X,Y,Z,YAW
    float vision_target_local_pos_est[4];
    float vision_target_global_pos_est[4];
    bool vision_target_pos_confirmed;

    PID_t pid[4];

    float vision_pos_coeff;
    float takeoff_height;
    float landing_height;
    float normal_altitude;

    bool use_guidance;
    bool freestyle;

protected:
    bool cmd_grabber(uint8_t c);
    uint8_t stat_grabber();
    bool control(uint8_t ground, float x, float y, float z, float yaw);
    bool pid_control(uint8_t ground, float x, float y, float z, float yaw);
    bool goal_reached();

public:
    uint8_t status();
    bool doStandby();
    bool doGrabBullets();
    bool doRequestControl();
    bool doTakeoff();
    bool doToNormalAltitude();
    bool doFlyToCar();
    bool doFindCar();
    bool doServeCar();
    bool doDropBullets();
    bool doBackToNormalAltitude();
    bool doFlyBack();
    bool doFindPark();
    bool doVisualServoLanding();
    bool doLanding();
    bool doReleaseControl();

    bool cmd_jet_state(uint8_t cmd);
    bool action(uint8_t cmd);
    void vision_cali();
    void stateMachine();
    void spin();

public:
    static void help();
};