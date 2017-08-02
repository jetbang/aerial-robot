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
 
#include "uav.h"

using namespace uav;

UAV::UAV() : node(~) : spinner(1) : debug(true)
{
    velocity_sub = node.subscribe("/guidance/velocity", 1, velocity_callback, this);
    ultrasonic_sub = node.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback, this);
    position_sub = node.subscribe("/guidance/position", 1, position_callback, this);
    vision_sub = node.subscribe("/vision/position", 1, vision_callback, this);

    drone = new DJIDrone(node);

    spinner.start();

    ROS_INFO("UAV Object Constrcuted");
}

UAV::~UAV()
{
    spinner.stop();
    if (drone != nullptr)
    {   
        delete drone;
        drone = NULL;
    }
    ROS_INFO("UAV Object Destrcuted");
}

void UAV::position_callback(const geometry_msgs::Vector3Stamped& position)
{
    g_pos = position;
    if (debug)
    {
        printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
        for (int i = 0; i < 5; i++)
            printf("global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
    }
}

void UAV::velocity_callback(const geometry_msgs::Vector3Stamped& velocity)
{
    g_vel = velocity;
    if (debug)
    {
        printf( "frame_id: %s stamp: %d\n", g_vel.header.frame_id.c_str(), g_vel.header.stamp.sec );
        printf( "velocity: [%f %f %f]\n", g_vel.vector.x, g_vel.vector.y, g_vel.vector.z );
    }
}

void UAV::vision_callback(const geometry_msgs::Vector3Stamped& position)
{
    v_pos = position;
    if (debug)
    {
        printf("frame_id: %s stamp: %d\n", v_pos.header.frame_id.c_str(), v_pos.header.stamp.sec);
        for (int i = 0; i < 5; i++)
            printf("vision position: [%f %f %f]\n", v_pos.vector.x, v_pos.vector.y, v_pos.vector.z);
    }
}

void UAV::ultrasonic_callback(const sensor_msgs::LaserScan& scan)
{
    g_scan = scan;
    if (debug)
    {
        printf( "frame_id: %s stamp: %d\n", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );
        for (int i = 0; i < 5; i++)
            printf( "ultrasonic distance: [%f]  reliability: [%d]\n", g_ul.ranges[i], (int)g_ul.intensities[i] );
    }
}

bool UAV::attach()
{
    bool result = drone->request_sdk_permission_control();
    if (debug)
    {
        printf( "UAV::attach result: %d\n", result);
    }
    return result;
}

bool UAV::detach()
{
    bool result = drone->release_sdk_permission_control();
    if (debug)
    {
        printf( "UAV::detach result: %d\n", result);
    }
    return result;
}

bool UAV::takeoff()
{
    bool result = drone->takeoff();
    if (debug)
    {
        printf( "UAV::takeoff result: %d\n", result);
    }
    return result;
}

bool UAV::landing()
{
    bool result = drone->landing();
    if (debug)
    {
        printf( "UAV::landing result: %d\n", result);
    }
    return result;
}

bool UAV::control(unsigned char flag, float x, float y, float z, float yaw)
{
    bool result = drone->attitude_control(flag, x, y, z, yaw);
    if (debug)
    {
        printf( "UAV::control result: %d\n", result);
    }
    return result;
}

bool UAV::guidance(float x, float y, float z, float yaw)
{
    return true;
}

bool UAV::drop_bullets()
{
    return true;
}
