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

#include "action.h"

using namespace uav;

bool Action::takeoff(const uav_action_plan::TakeoffGoalConstPtr& goal)
{

}

bool Action::landing(const uav_action_plan::LandingGoalConstPtr& goal)
{

}

bool Action::attitude(const uav_action_plan::AttitudeGoalConstPtr& goal)
{
  /*IMPORTANT*/
  /*
     There has been declared a pointer `local_navigation_action` as the function parameter,
     However, it is the `local_navigation_action_server` that we should use.
     If `local_navigation_action` is used instead, there will be a runtime sengmentation fault.

     so interesting
  */

  float dst_x = goal->x;
  float dst_y = goal->y;
  float dst_z = goal->z;
  float dst_yaw = goal->yaw;

  float org_x = local_position.x;
  float org_y = local_position.y;
  float org_z = local_position.z;
  float org_yaw = local_position.yaw;

  float dis_x = dst_x - org_x;
  float dis_y = dst_y - org_y;
  float dis_z = dst_z - org_z; 
  float dis_yaw = dst_yaw - org_yaw; 

  float det_x, det_y, det_z, det_yaw;

  DJI::onboardSDK::FlightData flight_ctrl_data;
  flight_ctrl_data.flag = 0x90;
  flight_ctrl_data.z = dst_z;
  flight_ctrl_data.yaw = 0;

  int x_progress = 0; 
  int y_progress = 0; 
  int z_progress = 0; 
  while (x_progress < 100 || y_progress < 100 || z_progress <100) {

     flight_ctrl_data.x = dst_x - local_position.x;
     flight_ctrl_data.y = dst_y - local_position.y;
     rosAdapter->flight->setMovementControl(flight_ctrl_data.flag, flight_ctrl_data.x, flight_ctrl_data.y, flight_ctrl_data.z, flight_ctrl_data.yaw);

     det_x = (100 * (dst_x - local_position.x)) / dis_x;
     det_y = (100 * (dst_y - local_position.y)) / dis_y;
     det_z = (100 * (dst_z - local_position.z)) / dis_z;

     x_progress = 100 - (int)det_x;
     y_progress = 100 - (int)det_y;
     z_progress = 100 - (int)det_z;

     //lazy evaluation
     if (std::abs(dst_x - local_position.x) < 0.1) x_progress = 100;
     if (std::abs(dst_y - local_position.y) < 0.1) y_progress = 100;
     if (std::abs(dst_z - local_position.z) < 0.1) z_progress = 100;

     attitude_feedback.x_progress = x_progress;
     attitude_feedback.y_progress = y_progress;
     attitude_feedback.z_progress = z_progress;
     attitude_action_server->publishFeedback(attitude_feedback);

     usleep(20000);
  }

  attitude_result.result = true;
  attitude_action_server->setSucceeded(attitude_result);

  return true;
}
