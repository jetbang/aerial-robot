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

#include "cfg.h"

void Cfg::fill_dropoint_param(geometry_msgs::Vector3& dropoint, const cv::FileNodeIterator& it)
{
    dropoint.x   = (double)(*it)["x"];
    dropoint.y   = (double)(*it)["y"];
    dropoint.z   = (double)(*it)["z"];

    std::cout << "dropoint:" << std::endl;
    std::cout << (double)(*it)["x"] << "  ";
    std::cout << (double)(*it)["y"] << "  ";
    std::cout << (double)(*it)["z"] << std::endl;
}

bool Cfg::load_dropoint_param(geometry_msgs::Vector3& dropoint)
{
    cv::FileStorage fs("src/uav_ros/config/dropoint.yaml", cv::FileStorage::READ);
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

void Cfg::fill_pid_param(PID_t* pid, const cv::FileNodeIterator& it)
{
    pid->kp = (float)(*it)["kp"];
    pid->ki = (float)(*it)["ki"];
    pid->kd = (float)(*it)["kd"];
    pid->db = (float)(*it)["db"];
    pid->it = (float)(*it)["it"];
    pid->Emax = (float)(*it)["Emax"];
    pid->Pmax = (float)(*it)["Pmax"];
    pid->Imax = (float)(*it)["Imax"];
    pid->Dmax = (float)(*it)["Dmax"];
    pid->Omax = (float)(*it)["Omax"];

    PID_Reset(pid);

    std::cout << "pid param:" << std::endl;
    std::cout << pid->kp << "  ";
    std::cout << pid->ki << "  ";
    std::cout << pid->kd << "  ";
    std::cout << pid->db << "  ";
    std::cout << pid->it << "  ";
    std::cout << pid->Emax << "  ";
    std::cout << pid->Pmax << "  ";
    std::cout << pid->Imax << "  ";
    std::cout << pid->Dmax << "  ";
    std::cout << pid->Omax << "  ";
    std::cout << std::endl;
}

bool Cfg::load_pid_param(PID_t* pid)
{
    cv::FileStorage fs("src/uav_ros/config/pid.yaml", cv::FileStorage::READ);
    if( !fs.isOpened()) // if we have file with parameters, read them
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