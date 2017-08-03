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
#include <opencv2/opencv.hpp>
#include <iostream>
#include <geometry_msgs/Vector3.h>
#include "pid.h"

class Cfg
{
public:
    static void fill_dropoint_param(geometry_msgs::Vector3& dropoint, const cv::FileNodeIterator& it);
    static void fill_pid_param(PID_t* pid, const cv::FileNodeIterator& it);
    static bool load_dropoint_param(geometry_msgs::Vector3& dropoint);
    static bool load_pid_param(PID_t* pid);
};