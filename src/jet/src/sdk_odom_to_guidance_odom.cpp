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

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

ros::Publisher* guidance_odom_pub_ptr;

void callback(const nav_msgs::Odometry& sdk_odom)
{
    nav_msgs::Odometry guidance_odom = sdk_odom;
    guidance_odom.pose.pose.position.z = -sdk_odom.pose.pose.position.z;
    guidance_odom_pub_ptr->publish(guidance_odom);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sdk_odom_to_guidance_odom");

    ros::NodeHandle nh;

    ros::Subscriber sdk_odom_sub = nh.subscribe("dji_sdk/odometry", 10, callback);
    ros::Publisher guidance_odom_sim = nh.advertise<nav_msgs::Odometry>("guidance/odom", 10);
    guidance_odom_pub_ptr = &guidance_odom_sim;

    ros::Rate rate(80);

    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }
}
