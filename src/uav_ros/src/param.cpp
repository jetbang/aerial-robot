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

#include "param.h"

using namespace uav_action_plan;

void Param::fill_dropoint_param(geometry_msgs::Vector3& dropoint, const cv::FileNodeIterator& it)
{
    dropoint.x   = (double)(*it)["x"];
    dropoint.y   = (double)(*it)["y"];
    dropoint.z   = (double)(*it)["z"];

    std::cout << (double)(*it)["x"] << "  ";
    std::cout << (double)(*it)["y"] << "  ";
    std::cout << (double)(*it)["z"] << std::endl;
}

bool Param::load_dropoint_param(geometry_msgs::Vector3& dropoint)
{
    cv::FileStorage fs("config/dropoint.yaml", cv::FileStorage::READ);
    if( !fs.isOpened() ) // if we have file with parameters, read them
    {
        std::cout<<"ERROR, cannot open dropoint.yaml!"<<std::endl;
    }
    cv::FileNode list_n = fs["dropoint"];
    cv::FileNodeIterator it = list_n.begin();
    fill_dropoint_param(dropoint, it);
    fs.release();
    return true;
}

void Param::fill_pid_param(PID_t* pid, const cv::FileNodeIterator& it)
{
    pid->kp = (float)(*it)["kp"];
    pid->ki = (float)(*it)["ki"];
    pid->kd = (float)(*it)["kd"];
    pid->db = (float)(*it)["db"];
    pid->it = (float)(*it)["it"];
    pid->EMax = (float)(*it)["EMax"];
    pid->PMax = (float)(*it)["PMax"];
    pid->IMax = (float)(*it)["IMax"];
    pid->DMax = (float)(*it)["DMax"];
    pid->OMax = (float)(*it)["OMax"];
}

bool Param::load_pid_param(PID_t* pid)
{
    cv::FileStorage fs("config/pid.yaml", cv::FileStorage::READ);
    if( !fs.isOpened() ) // if we have file with parameters, read them
    {
        std::cout<<"ERROR, cannot open pid.yaml!"<<std::endl;
    }
    cv::FileNode list_n = fs["pid"];
    cv::FileNodeIterator it = list_n.begin(), it_end = list_n.end();
    for (int i = 0; it != it_end; ++it)
    {
        fill_pid_param(&pid[i], it);
    }
    fs.release();
    return true;
}
