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

#include <opencv2/opencv.hpp>

#include <algorithm>
#include <string>
#include <vector>

#include <geometry_msgs/Vector3.h> //velocity

#include "pid.h"

namespace uav_action_plan
{
class Param
{
public:
    bool load_dropoint_param(geometry_msgs::Vector3& dropoint);
    bool load_pid_param(PID_t* pid);
protected:
    void fill_dropoint_param(geometry_msgs::Vector3& dropoint, const cv::FileNodeIterator& it);
    void fill_pid_param(PID_t* pid, const cv::FileNodeIterator& it);
}
}