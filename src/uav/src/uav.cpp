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

UAV::UAV(ros::NodeHandle n) : drone(n), ws(STAND_BY), uart_fd(-1)
{
    nh = n;

    nh.param<bool>("debug", debug, true); 
    nh.param<bool>("enable_step", enable_step, true); 
    nh.param<bool>("enable_claw", enable_claw, false); 
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

    uart_fd = -1;
    int ret = uart_open(&uart_fd, serial_port.c_str(), serial_baudrate, UART_OFLAG_WR);
    if (ret < 0) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , serial_port.c_str());
    }

    // load pre-configed dropoint
    load_dropoint_param();
    // load default pid parameters
    load_pid_param();

    // initialize publishers
    uav_state_pub = nh.advertise<std_msgs::UInt8>("/uav_state", 10);
    guidance_odom_calied_pub = nh.advertise<nav_msgs::Odometry>("/odom_calied", 10);

    // initialize subscribers
    odom_sub = nh.subscribe("/guidance/odom", 10, &UAV::guidance_odom_callback, this);
    vision_sub = nh.subscribe("/vision/position", 10, &UAV::vision_callback, this);

    // initialize actions
    guidance_nav_action_server = new GuidanceNavActionServer(nh,
            "guidance_nav_action",
            boost::bind(&UAV::guidance_nav_action_callback, this, _1), false);
    guidance_nav_action_server->start();

    // initialize services
    charge_service  = nh.advertiseService("/charge", &UAV::charge_callback, this);
    cmd_claw_service  = nh.advertiseService("/cmd_claw", &UAV::cmd_claw_callback, this);
    stat_claw_service  = nh.advertiseService("/stat_claw", &UAV::stat_claw_callback, this);
    reload_pid_param_service = nh.advertiseService("/reload_pid_param", &UAV::reload_pid_param_callback, this);
    reload_dropoint_param_service = nh.advertiseService("/reload_dropoint_param", &UAV::reload_dropoint_param_callback, this);
    reload_vision_param_service = nh.advertiseService("/reload_vision_param", &UAV::reload_vision_param_callback, this);
    
    ros::spinOnce();

    std::cout << "                                                                         \n       111                      111                                      \n       111                      111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111  1111111  111111111  11111111   1111111  111111111   11111111 \n       111 1111 1111   1111     1111 111  1111 111  11111 111  111 111   \n       111 111   111   1111     111  1111 11    111 1111  111  111  111  \n 111   111 111111111   1111     111   111    111111 111   111  111 1111  \n 111   111 111         1111     111   111 111111111 111   111  1111111   \n 111  1111 111         1111     111   111 111   111 111   111  111111    \n 111  111  1111  111    111     111  1111 111  1111 111   111  111       \n  1111111   11111111    111111  11111111  111111111 111   111  11111111  \n   11111     11111       11111  1111111    11111111 111   111  111  1111 \n                                                              111    111 \n                                                               11111111  " << std::endl;

    std::cout << "\n----------------------------Copyright 2017. Jetbang----------------------------" << std::endl;
}

void UAV::load_dropoint_param()
{
    nh.param<double>("uav/dropoint/x", dropoint.x, 0.0);
    nh.param<double>("uav/dropoint/y", dropoint.y, 0.0);
    nh.param<double>("uav/dropoint/z", dropoint.z, 0.0);
}

void UAV::fill_pid_param(int i, const char* axis)
{
    std::stringstream ss;
    ss << "uav/pid/" << axis << "/";
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

    float org_x = odom_calied.pose.pose.position.x;
    float org_y = odom_calied.pose.pose.position.y;
    float org_z = odom_calied.pose.pose.position.z;
    float org_yaw = tf::getYaw(odom_calied.pose.pose.orientation);

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

        float yaw = tf::getYaw(odom_calied.pose.pose.orientation);

        flight_ctrl_data.x = dst_x - odom_calied.pose.pose.position.x;
        flight_ctrl_data.y = dst_y - odom_calied.pose.pose.position.y;
        flight_ctrl_data.z = dst_z - odom_calied.pose.pose.position.z;
        flight_ctrl_data.yaw = dst_yaw - yaw;

        pid_control(1, flight_ctrl_data.x, flight_ctrl_data.y, flight_ctrl_data.z, flight_ctrl_data.yaw);

        det_x = (100 * (dst_x - odom_calied.pose.pose.position.x)) / dis_x;
        det_y = (100 * (dst_y - odom_calied.pose.pose.position.y)) / dis_y;
        det_z = (100 * (dst_z - odom_calied.pose.pose.position.z)) / dis_z;
        det_yaw = (100 * (dst_yaw - yaw)) / dis_yaw;

        x_progress = 100 - (int)det_x;
        y_progress = 100 - (int)det_y;
        z_progress = 100 - (int)det_z;
        yaw_progress = 100 - (int)det_yaw;

        //lazy evaluation
        if (std::abs(dst_x - odom_calied.pose.pose.position.x) < xy_err_tolerence) x_progress = 100;
        if (std::abs(dst_y - odom_calied.pose.pose.position.y) < xy_err_tolerence) y_progress = 100;
        if (std::abs(dst_z - odom_calied.pose.pose.position.z) < z_err_tolerence) z_progress = 100;
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

bool UAV::reload_vision_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    nh.param<float>("vision_pos_coeff", vision_pos_coeff, 0.001f);
    return true;
}

// 1  	standby
// 2 	take_off
// 3 	in_air
// 4 	landing
// 5 	finish_landing
uint8_t UAV::status()
{
    uint8_t result = drone.flight_status;
    if (debug)
    {
        printf( "UAV::status result: %d\n", result);
    }
    return result;
}

bool UAV::activate()
{
    bool result = drone.activate();
    if (debug)
    {
        printf( "UAV::activate result: %d\n", result);
    }
    return result;
}

bool UAV::request_control()
{
    bool result = drone.request_sdk_permission_control();
    if (debug)
    {
        printf( "UAV::request_control result: %d\n", result);
    }
    return result;
}

bool UAV::release_control()
{
    bool result = drone.release_sdk_permission_control();
    if (debug)
    {
        printf( "UAV::release_control result: %d\n", result);
    }
    return result;
}

bool UAV::takingoff()
{
    bool result = drone.takeoff();
    if (debug)
    {
        printf( "UAV::takeoff result: %d\n", result);
    }
    return result;
}

bool UAV::landing()
{
    bool result = drone.landing();
    if (debug)
    {
        printf( "UAV::landing result: %d\n", result);
    }
    return result;
}

bool UAV::control(unsigned char flag, float x, float y, float z, float yaw)
{
    bool result = drone.attitude_control(flag, x, y, z, yaw);
    //bool result = drone.velocity_control(flag, x, y, z, yaw);
    if (debug)
    {
        printf( "UAV::control result: %d\n", result);
    }
    return result;
}

bool UAV::pid_control(uint8_t frame = 0, float x = 0, float y = 0, float z = 0, float yaw = 0)
{

    float rx = odom_calied.pose.pose.position.x + x;
    float fx = odom_calied.pose.pose.position.x;
    PID_Calc(&pid[0], rx, fx);

    float ry = odom_calied.pose.pose.position.y + y;
    float fy = odom_calied.pose.pose.position.y;
    PID_Calc(&pid[1], ry, fy);

    float rz = odom_calied.pose.pose.position.z + z;
    float fz = odom_calied.pose.pose.position.z;
    PID_Calc(&pid[2], rz, fz);
    
    float g_yaw = tf::getYaw(odom_calied.pose.pose.orientation);
    float ryaw = g_yaw + yaw;
    float fyaw = g_yaw;
    PID_Calc(&pid[3], ryaw, fyaw);

    float x_thr = pid[0].out;
    float y_thr = pid[1].out;
    float z_thr = pid[2].out;
    float yaw_thr = pid[3].out;

    uint8_t flag = Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                        Flight::VerticalLogic::VERTICAL_VELOCITY |
                        Flight::YawLogic::YAW_RATE |
                        Flight::SmoothMode::SMOOTH_ENABLE;

/*
    uint8_t flag = Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                        Flight::VerticalLogic::VERTICAL_VELOCITY |
                        Flight::YawLogic::YAW_RATE |
                        Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
                        Flight::SmoothMode::SMOOTH_ENABLE;
*/
    //bool result = true;
    if (frame)
    {
        // WORLD
        flag |= Flight::HorizontalCoordinate::HORIZONTAL_GROUND;
    } else {
        // BODY
        flag |= Flight::HorizontalCoordinate::HORIZONTAL_BODY;
    }

    bool result = control(flag, x_thr, y_thr, z_thr, yaw_thr);

    std::cout << "rx: " << rx << ", ry: " << ry << ", rz: " << rz << ", ryaw: " << ryaw << std::endl;
    std::cout << "fx: " << fx << ", fy: " << fy << ", fz: " << fz << ", fyaw: " << fyaw << std::endl;
    std::cout << "ex: " << x << ", ey: " << y << ", ez: " << z << ", eyaw: " << yaw << std::endl;
    std::cout << "ox: " << x_thr << ", oy: " << y_thr << ", oz: " << z_thr << ", oyaw: " << yaw_thr << std::endl;

    return result;
}

bool UAV::cmd_claw(char c)
{
    if (!enable_claw)
    {
        claw_state = c;
        return true;
    }
    #define BUF_LEN 4
    uint8_t buf[BUF_LEN];
    buf[0] = 0xa5;
    buf[1] = c;
    buf[2] = 0xfe;
    CRC8Append(buf, BUF_LEN, 0xff);
    if (write(uart_fd, buf, BUF_LEN) == BUF_LEN)
    {
        claw_state = c;
        return true;
    }
    return false;
}

uint8_t UAV::stat_claw()
{
    if (!enable_claw)
    {
        return claw_state;
    }
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
    uint8_t status = this->status();

    static uint32_t counter = 0;

    // stand-by
    if (status == 1)
    {
        takingoff();
        return false;
    }
    // taking off
    else if (status == 2)
    {
        return false;
    }
    else if (status == 3) // && odom_calied.pose.pose.position.z < -takeoff_height)
    {
        if (counter < 350)
        {
            counter++;
            std::cout << "counter: " << counter << std::endl;
            return false;
        }
        else
        {
            counter = 0;
            return true;
        }
    }
    return false;
}

bool UAV::fly_to_car()
{
    float x_err = dropoint.x - odom_calied.pose.pose.position.x;
    float y_err = dropoint.y - odom_calied.pose.pose.position.y;
    // float z_err = dropoint.z - odom_calied.pose.pose.position.z;
    float z_err = 0;
    float yaw_err = 0;

    std::cout << "ex: " << x_err << ", ey: " << y_err << ", ez: " << z_err << ", eyaw: " << yaw_err << std::endl;
    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence && yaw_err < yaw_err_tolerence)
    {
        return true;
    }

    pid_control(1, x_err, y_err, z_err, yaw_err);

    return false;
}

bool UAV::serve_car()
{
    //float x_err = v_pos.vector.x * vision_pos_coeff;
    //float y_err = v_pos.vector.y * vision_pos_coeff;
    float x_err = dropoint.x - odom_calied.pose.pose.position.x;
    float y_err = dropoint.y - odom_calied.pose.pose.position.y;
    float z_err = dropoint.z - odom_calied.pose.pose.position.z;
    //float z_err = v_pos.vector.z - odom_calied.pose.pose.position.z;
    //float z_err = 0;
    float yaw_err = 0;

    std::cout << "ex: " << x_err << ", ey: " << y_err << ", ez: " << z_err << ", eyaw: " << yaw_err << std::endl;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence && yaw_err < yaw_err_tolerence)
    {
        return true;
    }

    pid_control(1, x_err, y_err, z_err, yaw_err);

    return false;
}

bool UAV::air_drop()
{
    if (!enable_claw)
    {
        return true;
    }
    if (stat_claw() == open_claw_cmd)
    {
        return true;
    }
    open_claw();
    return false;
}

bool UAV::ascend()
{
    float x_err = dropoint.x - odom_calied.pose.pose.position.x;
    float y_err = dropoint.y - odom_calied.pose.pose.position.y;
    float z_err = -takeoff_height - odom_calied.pose.pose.position.z;
    float yaw_err = 0;

    std::cout << "ex: " << x_err << ", ey: " << y_err << ", ez: " << z_err << ", eyaw: " << yaw_err << std::endl;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence && yaw_err < yaw_err_tolerence)
    {
        std::cout << "ascending back done" << std::endl;
        return true;
    }

    pid_control(1, x_err, y_err, z_err, yaw_err);

    return false;
}

bool UAV::fly_back()
{
    float x_err = 0 - odom_calied.pose.pose.position.x;
    float y_err = 0 - odom_calied.pose.pose.position.y;
    float z_err = -takeoff_height - odom_calied.pose.pose.position.z;
    float yaw_err = 0;

    std::cout << "ex: " << x_err << ", ey: " << y_err << ", ez: " << z_err << ", eyaw: " << yaw_err << std::endl;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence && yaw_err < yaw_err_tolerence)
    {
        std::cout << "flying back done" << std::endl;
        return true;
    }

    pid_control(1, x_err, y_err, z_err, yaw_err);

    return false;
}

bool UAV::serve_park()
{
    float x_err = v_pos.vector.x * vision_pos_coeff;
    float y_err = v_pos.vector.y * vision_pos_coeff;
    float z_err = -landing_height - odom_calied.pose.pose.position.z;
    float yaw_err = 0;

    std::cout << "ex: " << x_err << ", ey: " << y_err << ", ez: " << z_err << ", eyaw: " << yaw_err << std::endl;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence && yaw_err < yaw_err_tolerence)
    {
        std::cout << "serve parking done" << std::endl;
        return true;
    }

    pid_control(1, x_err, y_err, z_err);

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
        if (!enable_claw)
        {
            ws = REQUESTING_CONTROL;
        }
        else if (grab()) // && (enable_step ? kbhit() == 's' : true))
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
        if (request_control()) // && (enable_step ? kbhit() == 's' : true))
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
        if (takeoff()) // && (enable_step ? kbhit() == 's' : true))
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
        if (fly_to_car()) // && (enable_step ? kbhit() == 's' : true))
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
        if (serve_car()) // && (enable_step ? kbhit() == 's' : true))
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
        if (!enable_claw)
        {
            ws = ASCENDING;
        }
        else if (air_drop()) // && (enable_step ? kbhit() == 's' : true))
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
        if (ascend()) // && (enable_step ? kbhit() == 's' : true))
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
        if (fly_back()) // && (enable_step ? kbhit() == 's' : true))
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
        if (serve_park()) // && (enable_step ? kbhit() == 's' : true))
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
        if (land()) // && (enable_step ? kbhit() == 's' : true))
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
        if (release_control()) // && (enable_step ? kbhit() == 's' : true))
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

void UAV::console(uint8_t cmd)
{
    switch (cmd)
    {
        case STAND_BY:
        break;
        case GRABBING:
        grab();
        break;
        case REQUESTING_CONTROL:
        request_control();
        break;
        case TAKING_OFF:
        takeoff();
        break;
        case FLYING_TO_CAR:
        while(!fly_to_car());
        break;
        case SERVING_CAR:
        serve_car();
        break;
        case AIR_DROPPING:
        air_drop();
        break;
        case ASCENDING:
        ascend();
        case FLYING_BACK:
        fly_back();
        break;
        case SERVING_PARK:
        serve_park();
        break;
        case LANDING:
        land();
        break;
        case RELEASING_CONTROL:
        release_control();
        default:
        break;
    }
}

void UAV::publish_state()
{
    std_msgs::UInt8 msg;
    msg.data = ws;
    uav_state_pub.publish(msg);
}

static void help()
{
    std::cout << "\n        STAND_BY: \t0,\n        GRABBING: \t1,\n        REQUESTING_CONTROL: \t2,\n        TAKING_OFF: \t3,\n        FLYING_TO_CAR: \t4,\n        SERVING_CAR: \t5,\n        AIR_DROPPING: \t6,\n        ASCENDING: \t7,\n        FLYING_BACK: \t8,\n        SERVING_PARK: \t9,\n        LANDING: \ta,\n        RELEASING_CONTROL:\t b\n" << std::endl;
}

void UAV::spin()
{
    ros::Rate rate(spin_rate);
    uint8_t flag = Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                        Flight::VerticalLogic::VERTICAL_VELOCITY |
                        Flight::YawLogic::YAW_RATE |
                        Flight::HorizontalCoordinate::HORIZONTAL_GROUND |
                        Flight::SmoothMode::SMOOTH_ENABLE;
    while (ros::ok())
    {
        ros::spinOnce();
/*
        char kb = kbhit();
        if (kb >= '0' && kb <= '9')
        {
            console(kb - '0');
        }
        else if (kb >= 'a' && kb <= 'b')
        {
            console(kb - 'a' + 10);
        }
        else if (kb == 'h')
        {
            help();
        }
        else if (kb == 'w')
        {
            control(flag, 0, 1, 0, 0);
        }
        else if (kb == 's')
        {
            control(flag, 0, -1, 0, 0);
        }
        else if (kb == 'q')
        {
            control(flag, 1, 0, 0, 0);
        }
        else if (kb == 'e')
        {
            control(flag, -1, 0, 0, 0);
        }
        else if (kb == 'r')
        {
            control(flag, 0, 0, 0, 1);
        }
        else if (kb == 'f')
        {
            control(flag, 0, 0, 0, -1);
        }
*/
        stateMachine();

        printState();

        publish_state();

        rate.sleep();
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav");

    ros::NodeHandle nh;

    UAV uav(nh);
    
    /*
    uav.request_control();

    usleep(2e6);

    uav.takingoff();
    
    usleep(20e6);
    */

    uav.spin();

    std::cout << "                                                                         \n       111                      111                                      \n       111                      111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111             1111     111                                      \n       111  1111111  111111111  11111111   1111111  111111111   11111111 \n       111 1111 1111   1111     1111 111  1111 111  11111 111  111 111   \n       111 111   111   1111     111  1111 11    111 1111  111  111  111  \n 111   111 111111111   1111     111   111    111111 111   111  111 1111  \n 111   111 111         1111     111   111 111111111 111   111  1111111   \n 111  1111 111         1111     111   111 111   111 111   111  111111    \n 111  111  1111  111    111     111  1111 111  1111 111   111  111       \n  1111111   11111111    111111  11111111  111111111 111   111  11111111  \n   11111     11111       11111  1111111    11111111 111   111  111  1111 \n                                                              111    111 \n                                                               11111111  " << std::endl;

    return 0;
}


/*
static void Display_Main_Menu(void)
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > -------------------------+\n");
	printf("| [1]  SDK Version Query        | [2]  Request Control             |\n");
	printf("| [3]  Release Control          | [4]  Takeoff                     |\n");	
	printf("| [5]  Landing                  | [6]  Followme Mission Upload     |\n");	
    printf("input 1/2/3 etc..then press enter key\r\n");
    printf("use `rostopic echo` to query drone status\r\n");
    printf("----------------------------------------\r\n");
}

int main(int argc, char *argv[])
{
    int main_operate_code = 0;
    int temp32;

    ros::init(argc, argv, "uav");
    ROS_INFO("uav");

    ros::NodeHandle nh;

    //DJIDrone drone(nh);
    UAV uav(nh);

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

        if(temp32>0 && temp32<38)
        {
            main_operate_code = temp32;         
        }
        else
        {
            printf("ERROR - Out of range Input \n");
            Display_Main_Menu();
            continue;
        }

        switch(main_operate_code)
        {
			case 1:
				//drone.check_version();
				break;
            case 2:
                //drone.request_sdk_permission_control();
                uav.request_control();
                break;
            case 3:
                //drone.release_sdk_permission_control();
                uav.release_control();
                break;
            case 4:
                //drone.takeoff();
                uav.takingoff();
                break;
            case 5:
                //drone.landing();
                uav.landing();
                break;
        }
    }


    return 0;
}
*/

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

