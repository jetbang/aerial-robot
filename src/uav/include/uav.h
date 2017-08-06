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
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include<tf/transform_broadcaster.h>

#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <deque>

#include "uav/GuidanceNavAction.h"
#include "uav/Charge.h"
#include "uav/CmdClaw.h"
#include "uav/StatClaw.h"

#include "crc8.h"
#include "uart.h"
#include "pid.h"
#include "kbhit.h"

using namespace DJI::onboardSDK;

class UAV
{
public:
    UAV(); // Constructor
    ~UAV(); // Destructor

public:
    bool debug;
    bool enable_step;

    typedef enum Workstate
    {
        STAND_BY,
        GRABBING,
        REQUESTING_CONTROL,
        TAKING_OFF,
        FLYING_TO_CAR,
        SERVING_CAR,
        AIR_DROPPING,
        ASCENDING,
        FLYING_BACK,
        SERVING_PARK,
        LANDING,
        RELEASING_CONTROL,
    }Workstate;
    
public:
    DJIDrone* drone;

    nav_msgs::Odometry odom;
    nav_msgs::Odometry odom_bias;
    nav_msgs::Odometry odom_calied;

    geometry_msgs::Vector3Stamped v_pos; // Vision target relative position feedback

protected:
    ros::NodeHandle nh;
    
    // publishers
    ros::Publisher uav_state_pub;
    ros::Publisher guidance_odom_calied_pub;

    // subscribers
    ros::Subscriber odom_sub;
    ros::Subscriber vision_sub;    

    // actions
    typedef actionlib::SimpleActionServer<uav::GuidanceNavAction> GuidanceNavActionServer;

    GuidanceNavActionServer* guidance_nav_action_server;
    uav::GuidanceNavFeedback guidance_nav_feedback;
    uav::GuidanceNavResult guidance_nav_result;

    // services
    ros::ServiceServer charge_service;
    ros::ServiceServer cmd_claw_service;
    ros::ServiceServer stat_claw_service;
    ros::ServiceServer reload_pid_param_service;
    ros::ServiceServer reload_dropoint_param_service;


protected:
    // other member variables
    //uint8_t cflag; // control flags

    Workstate ws;

    geometry_msgs::Vector3 dropoint;
    PID_t pid[6];

    int uart_fd;
    std::string serial_port;
    int serial_baudrate;
    int open_claw_cmd;
    int close_claw_cmd;

    int spin_rate;

    float xy_err_tolerence;
    float z_err_tolerence;
    float yaw_err_tolerence;

    float vision_pos_coeff;
    float takeoff_height;
    float landing_height;

    int serial_timeout;
    int callback_timeout;

    bool calied;

    static const uint8_t LOST_COUNTER_NUM = 3;

    typedef enum
    {
        WDG_IDX_GUIDANCE,
        WDG_IDX_VISION,
        WDG_IDX_CLAW,
    } WdgIdx_e;

    uint32_t lost_counter[LOST_COUNTER_NUM];

public:
    void load_dropoint_param();
    void fill_pid_param(int i, const char* axis);
    void load_pid_param();
    void calc_odom_calied();
    void publish_odom_calied();

protected:
    
    // subscriber callbacks
    void guidance_odom_callback(const nav_msgs::OdometryConstPtr& g_odom);
    void vision_callback(const geometry_msgs::Vector3Stamped& position);

    // action callbacks
    bool guidance_nav_action_callback(const uav::GuidanceNavGoalConstPtr& goal);

    // service callbacks
    bool charge_callback(uav::Charge::Request& request, uav::Charge::Response& response);
    bool cmd_claw_callback(uav::CmdClaw::Request& request, uav::CmdClaw::Response& response);
    bool stat_claw_callback(uav::StatClaw::Request& request, uav::StatClaw::Response& response);
    bool reload_pid_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool reload_dropoint_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

public:
    // low level control api
    uint8_t status();
    bool activate();
    bool request_control();
    bool release_control();
    bool takingoff();
    bool landing();
    bool control(unsigned char flag, float x, float y, float z, float yaw);
    bool pid_control(float x, float y, float z, float yaw);
    bool cmd_claw(char c);
    bool open_claw();
    bool close_claw();
    uint8_t stat_claw();

public:
    // state machine process oriented api
    bool grab();
    bool takeoff();
    bool fly_to_car();
    bool serve_car();
    bool air_drop();
    bool ascend();
    bool fly_back();
    bool serve_park();
    bool land();

    // state machine logic
    void stateMachine(); // workstate machine
    void printState(); //
    void publish_state();

    // ros spin entrence
    void spin();
};
