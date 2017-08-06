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

#include "pid.h"
#include "uart.h"
#include "kbhit.h"

class Jet
{
public:
    typedef enum
    {
        GRAB_BULLETS,
        REQUEST_CONTROL,
        TAKE_OFF,
        FLY_TO_CAR,
        SERVE_CAR,
        DROP_BULLETS,
        BACK_TO_NORMAL_ALTITUDE,
        FLY_BACK,
        VISUAL_SERVO_LANDING,
        LANDING,
    } JetCmd_e;

public:
    Jet(ros::NodeHandle& nh);
    ~Jet();

protected:
    int uart_fd;
    int spin_rate;

    ros::NodeHandle nh;

    ros::Subscriber odometry_subscriber;
    ros::Subscriber vision_subscriber;

    ros::Publisher jet_state_pub;
    ros::Publisher odom_calied_pub;

    nav_msgs::Odometry odom;
    nav_msgs::Odometry odom_bias;
    nav_msgs::Odometry odom_calied;
    bool calied;
    geometry_msgs::Vector3Stamped vision_target_pos;

    geometry_msgs::Vector3 dropoint;

protected:
    void calc_odom_calied();
    void publish_odom_calied();

protected:
    // subscriber callbacks
    void odometry_callback(const nav_msgs::OdometryConstPtr& odometry);
    void vision_callback(const geometry_msgs::Vector3Stamped& position);

protected:
    DJIDrone drone;

protected:
    bool cmd_grabber(uint8_t c);
    uint8_t stat_grabber();
    bool control(uint8_t ground, float x, float y, float z, float yaw);
    bool pid_control(uint8_t ground, float x, float y, float z, float yaw);
    uint8_t status();

public:
    bool doGrabBullets();
    bool doRequestControl();
    bool doTakeoff();
    bool doFlyToCar();
    bool doServeCar();
    bool doDropBullets();
    bool doBackToNormalAltitude();
    bool doFlyBack();
    bool doVisualServoLanding();
    bool doLanding();

    bool action(uint8_t cmd);
    void stateMachine();
    void spin();

protected:
    PID_t pid[4];
    
    float vision_pos_coeff;
    float takeoff_height;
    float landing_height;

protected:
    void load_dropoint_param(ros::NodeHandle& nh);
    void fill_pid_param(ros::NodeHandle& nh, int i, const char* axis);
    void load_pid_param(ros::NodeHandle& nh);
};