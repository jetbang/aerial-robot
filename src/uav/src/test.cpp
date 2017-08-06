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
#include <stdio.h>
#include <dji_sdk/dji_drone.h>
#include <cstdlib>
#include <stdlib.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

using namespace DJI::onboardSDK;

DJIDrone* drone;

typedef enum
{
    HALF_AUTO,
    AUTO,
} UavMode_e;

uint8_t uav_mode = HALF_AUTO;

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
} UavCmd_e;

uint8_t uav_cmd = 0;

typedef enum
{
    STANDING_BY,
    GRABBING_BULLETS,
    REQUESTING_CONTROL,
    TAKING_OFF,
    FLYING_TO_CAR,
    SERVING_CAR,
    DROPPING_BULLETS,
    BACKING_TO_NORMAL_ALTITUDE,
    FLYING_BACK,
    VISUAL_SERVO_LANDING,
    LANDING,
} UavState_e;

uint8_t uav_state = STANDING_BY;

static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > -------------------------+\n");
	printf("| [1]  Grab bullets             | [2]  Request Control             |\n");
	printf("| [3]  Takeoff                  | [4]  Fly to Car                  |\n");	
	printf("| [4]  Serve Car                | [5]  Drop bullets                |\n");	
    printf("| [6]  Back to Normal Altitude  | [7]  Fly Back                    |\n");	
    printf("| [8]  Visual Servo Landing     | [9]  Land                        |\n");	
    printf("input 1/2/3 etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}

bool bulletsGrabbed()
{

}

void doGrabBullets()
{

}

void doRequestControl()
{
    drone->request_sdk_permission_control();
}

void doTakeoff()
{
    drone->takeoff();
}

void doFlyToCar()
{

}

void doServeCar()
{

}

void doDropBullets()
{

}

void doBackToNormalAltitude()
{

}

void doFlyBack()
{

}

void doVisualServoLanding()
{

}

void doLanding()
{
    drone->landing();
}

void action(uint8_t cmd)
{
    switch (cmd)
    {
        case GRAB_BULLETS:
        doGrabBullets();
        break;

        case REQUEST_CONTROL:
        doRequestControl();
        break;

        case TAKE_OFF:
        doTakeoff();
        break;

        case FLY_TO_CAR:
        doFlyToCar();
        break;

        case SERVE_CAR:
        doServeCar();
        break;

        case DROP_BULLETS:
        doDropBullets();
        break;

        case BACK_TO_NORMAL_ALTITUDE:
        doBackToNormalAltitude();
        break;

        case FLY_BACK:
        doFlyBack();
        break;

        case VISUAL_SERVO_LANDING:
        doVisualServoLanding();
        break;

        case LANDING:
        doLanding();
        break;
    }
}

void stateMachine()
{
    switch (state)
    {
        case STANDING_BY:
        if (cmd == GRAB_BULLETS)
        {
            if (!bulletsGrabbed())
            {
                doGrabBullets();
                state = GRABBING_BULLETS;
            }
            else
            {
                state = REQUESTING_CONTROL;
            }
        }
        else if (cmd == REQUEST_CONTROL)
        {
            doRequestControl();
        }
        else if (cmd == TAKE_OFF)
        {
            doTakeoff();
        }
        break;
    }
}

int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;

    ros::init(argc, argv, "uav");
    ROS_INFO("uav");

    ros::NodeHandle nh;
    DJIDrone* drone = new DJIDrone(nh);

    ros::spinOnce();

    //ros::Rate rate(30);
    Display_Main_Menu();
    
    while(1)
    {
        ros::spinOnce();
        std::cout << "Enter Input Val: ";
        while(!(std::cin >> temp32))
        {
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Invalid input.  Try again: ";
        }

        if(temp32 > 0 && temp32 < 38)
        {
            main_operate_code = temp32;         
        }
        else
        {
            printf("ERROR - Out of range Input \n");
            Display_Main_Menu();
            continue;
        }
        action(cmd);

    }

    delete drone;

    return 0;
}
