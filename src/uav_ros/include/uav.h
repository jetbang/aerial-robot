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
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

using namespace DJI::onboardSDK;

//#include "uav_action_plan/GetThereAction.h"

//typedef actionlib::SimpleActionServer<uav_action_plan::GetThereAction> uav::GetThereActionServer;
namespace uav
{
class UAV
{
public:
    UAV();

public:
    bool debug;
    
public:
    DJI::onboardSDK::DJIDrone* drone;

    geometry_msgs::Vector3Stamped g_pos;  // Guidance position (Odometry) feedback
    geometry_msgs::Vector3Stamped g_vel; // Guidance velocity feedback
    sensor_msgs::LaserScan g_scan; // Guidance ultrasonic scan feedback
    geometry_msgs::Vector3Stamped v_pos; // Vision target relative position feedback

protected:
    ros::NodeHandle node;
    ros::AsyncSpinner spinner;
    
    ros::Subscriber velocity_sub;
    ros::Subscriber ultrasonic_sub;
    ros::Subscriber position_sub;
    ros::Subscriber vision_sub;    

protected:
    void position_callback(const geometry_msgs::Vector3Stamped& position);
    void velocity_callback(const geometry_msgs::Vector3Stamped& velocity);
    void ultrasonic_callback(const sensor_msgs::LaserScan& scan);
    void vision_callback(const geometry_msgs::Vector3Stamped& position);

public:
    bool attach();
    bool detach();
    bool takeoff();
    bool landing();
    bool control(unsigned char flag, float x, float y, float z, float yaw);
    bool guidance(float x, float y, float z, float yaw);
    bool drop_bullets();
}
}