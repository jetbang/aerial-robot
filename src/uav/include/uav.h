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

#include<tf/transform_broadcaster.h>

#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

#include "uav/Charge.h"

#include "crc8.h"
#include "uart.h"
#include "pid.h"
#include "cfg.h"

//typedef actionlib::SimpleActionServer<uav::GuidanceAction> uav::GuidanceActionServer;

class UAV
{
public:
    UAV(); // Constructor
    ~UAV(); // Destructor

public:
    bool debug;

    typedef enum Workstate
    {
        STAND_BY,
        GRABBING,
        TAKING_OFF,
        FLYING_TO_CAR,
        SERVING_CAR,
        AIR_DROPPING,
        ASCENDING,
        FLYING_BACK,
        SERVING_PARK,
        LANDING,
    }Workstate;
    
public:
    DJIDrone* drone;

    geometry_msgs::Vector3Stamped g_pos;  // Guidance position (Odometry) feedback
    geometry_msgs::Vector3Stamped g_vel; // Guidance velocity feedback
    sensor_msgs::LaserScan g_scan; // Guidance ultrasonic scan feedback
    geometry_msgs::Vector3Stamped v_pos; // Vision target relative position feedback

protected:
    ros::NodeHandle nh;
    
    ros::Subscriber velocity_sub;
    ros::Subscriber ultrasonic_sub;
    ros::Subscriber position_sub;
    ros::Subscriber vision_sub;    

    ros::ServiceServer charge_service;
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


protected:
    void position_callback(const geometry_msgs::Vector3Stamped& position);
    void velocity_callback(const geometry_msgs::Vector3Stamped& velocity);
    void ultrasonic_callback(const sensor_msgs::LaserScan& scan);
    void vision_callback(const geometry_msgs::Vector3Stamped& position);
    bool charge_service_callback(uav::Charge::Request& request, uav::Charge::Response& response);

public:
    bool attach();
    bool detach();
    bool takingoff();
    bool landing();
    bool control(unsigned char flag, float x, float y, float z, float yaw);
    bool reduce_err(float x, float y, float z, float yaw);
    bool cmd_claw(char c);
    bool open_claw();
    bool close_claw();

    bool grab();
    bool takeoff();
    bool fly_to_car();
    bool serve_car();
    bool air_drop();
    bool ascend();
    bool fly_back();
    bool serve_park();
    bool land();
    
    void stateMachine(); // workstate machine
    void spin();
};
