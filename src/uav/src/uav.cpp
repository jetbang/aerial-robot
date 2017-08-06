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

UAV::UAV() : nh("~"), ws(STAND_BY)
{
    nh.param<bool>("debug", debug, true); 
    nh.param<bool>("enable_step", enable_step, true); 
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
    nh.param<int>("serial_baudrate", serial_baudrate, 115200);
    nh.param<int>("open_claw_cmd", open_claw_cmd, 0);
    nh.param<int>("close_claw_cmd", close_claw_cmd, 1);
    nh.param<int>("spin_rate", spin_rate, 50);
    nh.param<float>("xy_err_tolerence", xy_err_tolerence, 0.1f);
    nh.param<float>("z_err_tolerence", z_err_tolerence, 0.1f);
    nh.param<float>("yaw_err_tolerence", yaw_err_tolerence, 0.1f);
    nh.param<float>("vision_pos_coeff", vision_pos_coeff, 0.001f);
    nh.param<float>("takeoff_height", takeoff_height, 1.0f);
    nh.param<float>("landing_height", landing_height, 0.3f);
    nh.param<int>("serial_timeout", serial_timeout, 200);
    nh.param<int>("callback_timeout", callback_timeout, 200);

    int ret = uart_open(&uart_fd, serial_port.c_str(), serial_baudrate, UART_OFLAG_RD);
    if (ret < 0) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , serial_port.c_str());
    }

    // load pre-configed dropoint
    load_dropoint_param();
    // load default pid parameters
    load_pid_param();

    ros::NodeHandle n;

    // initialize publishers
    uav_state_pub = n.advertise<std_msgs::UInt8>("/uav_state", 10);
    guidance_odom_calied_pub = nh.advertise<nav_msgs::Odometry>("/odom_calied", 10);

    // initialize subscribers
    odom_sub = n.subscribe("/guidance/odom", 10, &UAV::guidance_odom_callback, this);
    vision_sub = n.subscribe("/vision/position", 10, &UAV::vision_callback, this);

    // initialize actions
    guidance_nav_action_server = new GuidanceNavActionServer(n,
            "guidance_nav_action",
            boost::bind(&UAV::guidance_nav_action_callback, this, _1), false);
    guidance_nav_action_server->start();

    // initialize services
    charge_service  = nh.advertiseService("/charge", &UAV::charge_callback, this);
    cmd_claw_service  = nh.advertiseService("/cmd_claw", &UAV::cmd_claw_callback, this);
    stat_claw_service  = nh.advertiseService("/stat_claw", &UAV::stat_claw_callback, this);
    reload_pid_param_service = nh.advertiseService("/reload_pid_param", &UAV::reload_pid_param_callback, this);
    reload_dropoint_param_service = nh.advertiseService("/reload_dropoint_param", &UAV::reload_dropoint_param_callback, this);

    // initialize dji drone
    drone = new DJIDrone(nh);

/*
    cflag = Flight::HorizontalLogic::HORIZONTAL_POSITION |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_ANGLE |
                            Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                            Flight::SmoothMode::SMOOTH_ENABLE;
*/

    std::cout << "                                                                         \n       111                      111                                      \n       111                      111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111  1111111  111111111  11111111   1111111  111111111   11111111 \n       111 1111 1111   1111     1111 111  1111 111  11111 111  111 111   \n       111 111   111   1111     111  1111 11    111 1111  111  111  111  \n 111   111 111111111   1111     111   111    111111 111   111  111 1111  \n 111   111 111         1111     111   111 111111111 111   111  1111111   \n 111  1111 111         1111     111   111 111   111 111   111  111111    \n 111  111  1111  111    111     111  1111 111  1111 111   111  111       \n  1111111   11111111    111111  11111111  111111111 111   111  11111111  \n   11111     11111       11111  1111111    11111111 111   111  111  1111 \n                                                              111    111 \n                                                               11111111  " << std::endl;

    std::cout << "\n----------------------------Copyright 2017. Jetbang----------------------------" << std::endl;
}

void UAV::load_dropoint_param()
{
    nh.param<double>("dropoint/x", dropoint.x, 0.0);
    nh.param<double>("dropoint/y", dropoint.y, 0.0);
    nh.param<double>("dropoint/z", dropoint.z, 0.0);
}

void UAV::fill_pid_param(int i, const char* axis)
{
    std::stringstream ss;
    ss << "pid/" << axis << "/";
    std::string root = ss.str();
    nh.param<float>(root + "kp", pid[i].kp, 0.0f);
    nh.param<float>(root + "ki", pid[i].ki, 0.0f);
    nh.param<float>(root + "kd", pid[i].kd, 0.0f);
    nh.param<float>(root + "db", pid[i].db, 0.0f);
    nh.param<float>(root + "it", pid[i].it, 0.0f);
    nh.param<float>(root + "Emax", pid[i].Emax, 0.0f);
    nh.param<float>(root + "Pmax", pid[i].Pmax, 0.0f);
    nh.param<float>(root + "Imax", pid[i].Imax, 0.0f);
    nh.param<float>(root + "Dmax", pid[i].Dmax, 0.0f);
    nh.param<float>(root + "Omax", pid[i].Omax, 0.0f);
}

void UAV::load_pid_param()
{
    fill_pid_param(0, "xy");
    fill_pid_param(1, "xy");
    fill_pid_param(2, "z");
    fill_pid_param(3, "yaw");
}

UAV::~UAV()
{
    if (drone != NULL)
    {   
        delete drone;
        drone = NULL;
    }
    if (guidance_nav_action_server != NULL)
    {
        delete guidance_nav_action_server;
        guidance_nav_action_server = NULL;
    }
    if (uart_fd > 0)
    {
        uart_close(uart_fd);
        uart_fd = -1;
    }
}

void UAV::calc_odom_calied()
{
    double yaw = tf::getYaw(odom.pose.pose.orientation);
    double yaw_bias = tf::getYaw(odom_bias.pose.pose.orientation);
    double yaw_calied = yaw - yaw_bias;
    geometry_msgs::Quaternion quat_calied = tf::createQuaternionMsgFromYaw(yaw_calied);

    odom_calied.header = odom.header;
    odom_calied.child_frame_id = odom.child_frame_id;
    odom_calied.pose.pose.position.x = odom.pose.pose.position.x - odom_bias.pose.pose.position.x;
    odom_calied.pose.pose.position.y = odom.pose.pose.position.y - odom_bias.pose.pose.position.y;
    odom_calied.pose.pose.position.z = odom.pose.pose.position.z - odom_bias.pose.pose.position.z;
    odom_calied.pose.pose.orientation = quat_calied;
    odom_calied.twist = odom.twist;
}

void UAV::publish_odom_calied()
{
    guidance_odom_calied_pub.publish(odom_calied);
}

void UAV::guidance_odom_callback(const nav_msgs::OdometryConstPtr& g_odom)
{
    odom = *g_odom;
    if (calied == false)
    {
        odom_bias = odom;
        calied = true;
    }
    calc_odom_calied();
    publish_odom_calied();
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

bool UAV::guidance_nav_action_callback(const uav::GuidanceNavGoalConstPtr& goal)
{
    float dst_x = goal->x;
    float dst_y = goal->y;
    float dst_z = goal->z;
    float dst_yaw = goal->yaw;

    float org_x = odom.pose.pose.position.x;
    float org_y = odom.pose.pose.position.y;
    float org_z = odom.pose.pose.position.z;
    float org_yaw = tf::getYaw(odom.pose.pose.orientation);

    float dis_x = dst_x - org_x;
    float dis_y = dst_y - org_y;
    float dis_z = dst_z - org_z; 
    float dis_yaw = dst_yaw - org_yaw;

    float det_x, det_y, det_z, det_yaw;

    DJI::onboardSDK::FlightData flight_ctrl_data;
    flight_ctrl_data.flag = Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                            Flight::VerticalLogic::VERTICAL_VELOCITY |
                            Flight::YawLogic::YAW_RATE |
                            Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
                            Flight::SmoothMode::SMOOTH_ENABLE;
    
    int x_progress = 0; 
    int y_progress = 0; 
    int z_progress = 0; 
    int yaw_progress = 0;

    while (x_progress < 100 || y_progress < 100 || z_progress < 100 || yaw_progress < 100) {

        float yaw = tf::getYaw(odom.pose.pose.orientation);

        flight_ctrl_data.x = dst_x - odom.pose.pose.position.x;
        flight_ctrl_data.y = dst_y - odom.pose.pose.position.y;
        flight_ctrl_data.z = dst_z - odom.pose.pose.position.z;
        flight_ctrl_data.yaw = dst_yaw - yaw;

        pid_control(flight_ctrl_data.x, flight_ctrl_data.y, flight_ctrl_data.z, flight_ctrl_data.yaw);

        det_x = (100 * (dst_x - odom.pose.pose.position.x)) / dis_x;
        det_y = (100 * (dst_y - odom.pose.pose.position.y)) / dis_y;
        det_z = (100 * (dst_z - odom.pose.pose.position.z)) / dis_z;
        det_yaw = (100 * (dst_yaw - yaw)) / dis_yaw;

        x_progress = 100 - (int)det_x;
        y_progress = 100 - (int)det_y;
        z_progress = 100 - (int)det_z;
        yaw_progress = 100 - (int)det_yaw;

        //lazy evaluation
        if (std::abs(dst_x - odom.pose.pose.position.x) < xy_err_tolerence) x_progress = 100;
        if (std::abs(dst_y - odom.pose.pose.position.y) < xy_err_tolerence) y_progress = 100;
        if (std::abs(dst_z - odom.pose.pose.position.z) < z_err_tolerence) z_progress = 100;
        if (std::abs(dst_yaw - yaw) < yaw_err_tolerence) yaw_progress = 100;

        guidance_nav_feedback.x_progress = x_progress;
        guidance_nav_feedback.y_progress = y_progress;
        guidance_nav_feedback.z_progress = z_progress;
        guidance_nav_feedback.yaw_progress = yaw_progress;

        guidance_nav_action_server->publishFeedback(guidance_nav_feedback);

        usleep(20000);
    }

    guidance_nav_result.result = true;
    guidance_nav_action_server->setSucceeded(guidance_nav_result);

    return true;
}

bool UAV::charge_callback(uav::Charge::Request& request, uav::Charge::Response& response)
{
    if (ws == STAND_BY)
    {
        ws = GRABBING;
    }
    response.result = ws;
    return true;
}

bool UAV::cmd_claw_callback(uav::CmdClaw::Request& request, uav::CmdClaw::Response& response)
{
    uint8_t cmd = request.cmd;
    cmd_claw(cmd);
    uint8_t claw_stat = stat_claw();
    response.result = claw_stat;
    return true;
}

bool UAV::stat_claw_callback(uav::StatClaw::Request& request, uav::StatClaw::Response& response)
{
    uint8_t claw_stat = stat_claw();
    response.result = claw_stat;
    return true;
}

bool UAV::reload_pid_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_pid_param();
    return true;
}

bool UAV::reload_dropoint_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_dropoint_param();
    return true;
}

// 1  	standby
// 2 	take_off
// 3 	in_air
// 4 	landing
// 5 	finish_landing
uint8_t UAV::status()
{
    uint8_t result = drone->flight_status;
    if (debug)
    {
        printf( "UAV::status result: %d\n", result);
    }
    return result;
}

bool UAV::activate()
{
    bool result = drone->activate();
    if (debug)
    {
        printf( "UAV::activate result: %d\n", result);
    }
    return result;
}

bool UAV::request_control()
{
    bool result = drone->request_sdk_permission_control();
    if (debug)
    {
        printf( "UAV::request_control result: %d\n", result);
    }
    return result;
}

bool UAV::release_control()
{
    bool result = drone->release_sdk_permission_control();
    if (debug)
    {
        printf( "UAV::release_control result: %d\n", result);
    }
    return result;
}

bool UAV::takingoff()
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

bool UAV::pid_control(float x = 0, float y = 0, float z = 0, float yaw = 0)
{
    float ref = 0;
    float fdb = 0;

    ref = odom_calied.pose.pose.position.x + x;
    fdb = odom_calied.pose.pose.position.x;
    PID_Calc(&pid[0], ref, fdb);

    ref = odom_calied.pose.pose.position.y + y;
    fdb = odom_calied.pose.pose.position.y;
    PID_Calc(&pid[1], ref, fdb);

    ref = odom_calied.pose.pose.position.z + z;
    fdb = odom_calied.pose.pose.position.z;
    PID_Calc(&pid[2], ref, fdb);
    
    float g_yaw = tf::getYaw(odom_calied.pose.pose.orientation);
    ref = g_yaw + yaw;
    fdb = g_yaw;
    PID_Calc(&pid[3], ref, fdb);

    float x_thr = pid[0].out;
    float y_thr = pid[1].out;
    float z_thr = pid[2].out;
    float yaw_thr = pid[3].out;

    uint8_t flag = Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                        Flight::VerticalLogic::VERTICAL_VELOCITY |
                        Flight::YawLogic::YAW_RATE |
                        Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
                        Flight::SmoothMode::SMOOTH_ENABLE;

    bool result = control(flag, x_thr, y_thr, z_thr, yaw_thr);

    return result;
}

bool UAV::cmd_claw(char c)
{
    #define BUF_LEN 4
    uint8_t buf[BUF_LEN];
    buf[0] = 0xa5;
    buf[1] = c;
    buf[2] = 0xfe;
    CRC8Append(buf, BUF_LEN, 0xff);
    return write(uart_fd, buf, BUF_LEN) == BUF_LEN;
}

uint8_t UAV::stat_claw()
{
    #define BUF_LEN 4
    uint8_t buf[BUF_LEN];
    if (read(uart_fd, buf, BUF_LEN) != BUF_LEN)
    {
        return 0xff;
    }
    if (buf[0] == 0xa5 && buf[2] == 0xfe && CRC8Check(buf, BUF_LEN, 0xff))
    {
        return buf[1];
    }
    return 0xff;
}

bool UAV::open_claw()
{
    return cmd_claw(open_claw_cmd);
}

bool UAV::close_claw()
{
    return cmd_claw(close_claw_cmd);
}

// state machine process
bool UAV::grab()
{
    if (stat_claw() == close_claw_cmd)
    {
        return true;
    }
    close_claw();
    return false;
}

bool UAV::takeoff()
{
    if (status() == 3 && odom_calied.pose.pose.position.z < -takeoff_height)
    {
        return true;
    }
    takingoff();
    return false;
}

bool UAV::fly_to_car()
{
    float x_err = dropoint.x - odom_calied.pose.pose.position.x;
    float y_err = dropoint.y - odom_calied.pose.pose.position.y;
    // float z_err = dropoint.z - odom_calied.pose.pose.position.z;
    float z_err = 0;
    float yaw_err = 0;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence && yaw_err < yaw_err_tolerence)
    {
        return true;
    }

    pid_control(x_err, y_err, z_err, yaw_err);

    return false;
}

bool UAV::serve_car()
{
    float x_err = v_pos.vector.x * vision_pos_coeff;
    float y_err = v_pos.vector.y * vision_pos_coeff;
    float z_err = dropoint.z - odom_calied.pose.pose.position.z;
    float yaw_err = 0;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence && yaw_err < yaw_err_tolerence)
    {
        return true;
    }

    pid_control(x_err, y_err, z_err, yaw_err);

    return false;
}

bool UAV::air_drop()
{
    if (stat_claw() == open_claw_cmd)
    {
        return true;
    }
    open_claw();
    return false;
}

bool UAV::ascend()
{
    float x_err = 0;
    float y_err = 0;
    float z_err = -takeoff_height - odom_calied.pose.pose.position.z;
    float yaw_err = 0;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence && yaw_err < yaw_err_tolerence)
    {
        return true;
    }

    pid_control(x_err, y_err, z_err, yaw_err);

    return false;
}

bool UAV::fly_back()
{
    float x_err = odom_calied.pose.pose.position.x;
    float y_err = odom_calied.pose.pose.position.y;
    float z_err = 0;
    float yaw_err = 0;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence && yaw_err < yaw_err_tolerence)
    {
        return true;
    }

    pid_control(x_err, y_err, z_err);

    return false;
}

bool UAV::serve_park()
{
    float x_err = v_pos.vector.x * vision_pos_coeff;
    float y_err = v_pos.vector.y * vision_pos_coeff;
    float z_err = -landing_height - odom_calied.pose.pose.position.z;
    float yaw_err = 0;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence && yaw_err < yaw_err_tolerence)
    {
        return true;
    }

    pid_control(x_err, y_err, z_err);

    return false;
}

bool UAV::land()
{
    if (status() == 5)
    {
        return true;
    }
    landing();
    return false;
}

void UAV::printState()
{
    switch (ws)
    {
        case STAND_BY:
        std::cout << "stateMachine: stand-by " << std::endl;
        break;
        case GRABBING:
        std::cout << "stateMachine: grabing " << std::endl;
        break;
        case REQUESTING_CONTROL:
        std::cout << "stateMachine: requesting control " << std::endl;
        break;
        case TAKING_OFF:
        std::cout << "stateMachine: taking off " << std::endl;
        break;
        case FLYING_TO_CAR:
        std::cout << "stateMachine: flying to car " << std::endl;
        break;
        case SERVING_CAR:
        std::cout << "stateMachine: serving car " << std::endl;
        break;
        case AIR_DROPPING:
        std::cout << "stateMachine: air-droping " << std::endl;
        break;
        case ASCENDING:
        std::cout << "stateMachine: ascending " << std::endl;
        break;
        case FLYING_BACK:
        std::cout << "stateMachine: flying back " << std::endl;
        break;
        case SERVING_PARK:
        std::cout << "stateMachine: serving park " << std::endl;
        break;
        case LANDING:
        std::cout << "stateMachine: landing " << std::endl;
        break;
        case RELEASING_CONTROL:
        std::cout << "stateMachine: releasing control " << std::endl;
        break;
        default:
        std::cout << "stateMachine: undefined state " << std::endl;
        break;
    }
}

void UAV::stateMachine()
{
    switch (ws)
    {
        case STAND_BY:
        if (enable_step)
        {
            if (kbhit() == 's' || kbhit() == 'f')
            {
                ws = GRABBING;
            }
        }
        break;
        case GRABBING:
        if (grab() && (enable_step ? kbhit() == 's' : true))
        {
            ws = REQUESTING_CONTROL;
        }
        else if (enable_step && kbhit() == 'f')
        {
            ws = REQUESTING_CONTROL;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        case REQUESTING_CONTROL:
        if (request_control() && (enable_step ? kbhit() == 's' : true))
        {
            ws = TAKING_OFF;
        }
        else if (enable_step && kbhit() == 'f')
        {
            ws = TAKING_OFF;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        case TAKING_OFF:
        if (takeoff() && (enable_step ? kbhit() == 's' : true))
        {
            ws = FLYING_TO_CAR;
        }
        else if (enable_step && kbhit() == 'f')
        {
            ws = FLYING_TO_CAR;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        case FLYING_TO_CAR:
        if (fly_to_car() && (enable_step ? kbhit() == 's' : true))
        {
            ws = SERVING_CAR;
        }
        else if (enable_step && kbhit() == 'f')
        {
            ws = SERVING_CAR;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        case SERVING_CAR:
        if (serve_car() && (enable_step ? kbhit() == 's' : true))
        {
            ws = AIR_DROPPING;
        }
        else if (enable_step && kbhit() == 'f')
        {
            ws = AIR_DROPPING;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        case AIR_DROPPING:
        if (air_drop() && (enable_step ? kbhit() == 's' : true))
        {
            ws = ASCENDING;
        }
        else if (enable_step && kbhit() == 'f')
        {
            ws = ASCENDING;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        case ASCENDING:
        if (ascend() && (enable_step ? kbhit() == 's' : true))
        {
            ws = FLYING_BACK;
        }
        else if (enable_step && kbhit() == 'f')
        {
            ws = FLYING_BACK;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        case FLYING_BACK:
        if (fly_back() && (enable_step ? kbhit() == 's' : true))
        {
            ws = SERVING_PARK;
        }
        else if (enable_step && kbhit() == 'f')
        {
            ws = SERVING_PARK;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        case SERVING_PARK:
        if (serve_park() && (enable_step ? kbhit() == 's' : true))
        {
            ws = LANDING;
        }
        else if (enable_step && kbhit() == 'f')
        {
            ws = LANDING;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        case LANDING:
        if (land() && (enable_step ? kbhit() == 's' : true))
        {
            calied = false;
            ws = RELEASING_CONTROL;
        }
        else if (enable_step && kbhit() == 'f')
        {
            calied = false;
            ws = RELEASING_CONTROL;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        case RELEASING_CONTROL:
        if (release_control() && (enable_step ? kbhit() == 's' : true))
        {
            ws = STAND_BY;
        }
        else if (enable_step && kbhit() == 'f')
        {
            ws = STAND_BY;
        }
        else if (enable_step && kbhit() == 'r')
        {
            ws = STAND_BY;
        }
        break;
        default:
        ws = STAND_BY;
        break;
    }
}

void UAV::publish_state()
{
    std_msgs::UInt8 msg;
    msg.data = ws;
    uav_state_pub.publish(msg);
}

void UAV::spin()
{
    ros::Rate rate(spin_rate);
    while (ros::ok())
    {
        ros::spinOnce();

        stateMachine();

        printState();

        publish_state();

        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav");

    UAV uav;
    uav.spin();

    std::cout << "                                                                         \n       111                      111                                      \n       111                      111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111  1111111  111111111  11111111   1111111  111111111   11111111 \n       111 1111 1111   1111     1111 111  1111 111  11111 111  111 111   \n       111 111   111   1111     111  1111 11    111 1111  111  111  111  \n 111   111 111111111   1111     111   111    111111 111   111  111 1111  \n 111   111 111         1111     111   111 111111111 111   111  1111111   \n 111  1111 111         1111     111   111 111   111 111   111  111111    \n 111  111  1111  111    111     111  1111 111  1111 111   111  111       \n  1111111   11111111    111111  11111111  111111111 111   111  11111111  \n   11111     11111       11111  1111111    11111111 111   111  111  1111 \n                                                              111    111 \n                                                               11111111  " << std::endl;

    return 0;
}
