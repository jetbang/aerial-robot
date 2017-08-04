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

UAV::UAV() : nh("~"), debug(true), ws(STAND_BY), xy_err_tolerence(0.1f), z_err_tolerence(0.1f),
    uart_fd(-1), serial_port("/dev/ttyTHS2"), serial_baudrate(115200), spin_rate(50), 
    open_claw_cmd(0), close_claw_cmd(1), calied(false), vision_pos_coeff(0.001f), 
    takeoff_height(1.0f), landing_height(0.3f), serial_timeout(200), callback_timeout(200),
    serial_comm_timer(callback_timeout), position_callback_timer(callback_timeout), 
    velocity_callback_timer(callback_timeout), vision_callback_timer(callback_timeout), 
    ultrasonic_callback_timer(callback_timeout)
{
    ros::NodeHandle np("~");
    np.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
    np.param<int>("serial_baudrate", serial_baudrate, 115200);
    np.param<int>("open_claw_cmd", open_claw_cmd, 0);
    np.param<int>("close_claw_cmd", close_claw_cmd, 1);
    np.param<int>("spin_rate", spin_rate, 50);
    np.param<float>("xy_err_tolerence", xy_err_tolerence, 0.1f);
    np.param<float>("z_err_tolerence", z_err_tolerence, 0.1f);
    np.param<float>("vision_pos_coeff", vision_pos_coeff, 0.001f);
    np.param<float>("takeoff_height", takeoff_height, 1.0f);
    np.param<float>("landing_height", landing_height, 0.3f);
    np.param<int>("serial_timeout", serial_timeout, 200);
    np.param<int>("callback_timeout", callback_timeout, 200);

    serial_comm_timer.reset(serial_timeout);
    position_callback_timer.reset(callback_timeout);
    velocity_callback_timer.reset(callback_timeout);
    ultrasonic_callback_timer.reset(callback_timeout);
    vision_callback_timer.reset(callback_timeout);

    int ret = uart_open(&uart_fd, serial_port.c_str(), serial_baudrate, UART_OFLAG_RD);
    if (ret < 0) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , serial_port.c_str());
        throw ret;
    }

    // load pre-configed dropoint
    Cfg::load_dropoint_param(dropoint);
    // load default pid parameters
    Cfg::load_pid_param(pid);

    // initialize subscribers
    position_sub = nh.subscribe("/uav/position", 1, &UAV::position_callback, this);
    velocity_sub = nh.subscribe("/uav/velocity", 1, &UAV::velocity_callback, this);
    ultrasonic_sub = nh.subscribe("/guidance/ultrasonic", 1, &UAV::ultrasonic_callback, this);
    vision_sub = nh.subscribe("/vision/position", 1, &UAV::vision_callback, this);

    // initialize actions
    guidance_nav_action_server = new GuidanceNavActionServer(nh,
            "guidance_nav_action",
            boost::bind(&UAV::guidance_nav_action_callback, this, _1), false);
    guidance_nav_action_server->start();

    // initialize services
    charge_service  = nh.advertiseService("charge", &UAV::charge_callback, this);
    cmd_claw_service  = nh.advertiseService("cmd_claw", &UAV::cmd_claw_callback, this);
    stat_claw_service  = nh.advertiseService("stat_claw", &UAV::stat_claw_callback, this);

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

UAV::~UAV()
{
    if (drone != NULL)
    {   
        delete drone;
        drone = NULL;
    }
    if (uart_fd > 0)
    {
        uart_close(uart_fd);
        uart_fd = -1;
    }
}

void UAV::position_callback(const geometry_msgs::Vector3Stamped& position)
{
    position_callback_timer.reset(callback_timeout);
    g_pos = position;
    if (!calied)
    {
        g_pos_bias = g_pos;
        calied = true;
    }
    g_pos_calied.header =  g_pos.header;
    g_pos_calied.vector.x = g_pos.vector.x - g_pos_bias.vector.x;
    g_pos_calied.vector.y = g_pos.vector.y - g_pos_bias.vector.y;
    g_pos_calied.vector.z = g_pos.vector.z - g_pos_bias.vector.z;
    if (debug)
    {
        printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
        for (int i = 0; i < 5; i++)
            printf("global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
    }
}

void UAV::velocity_callback(const geometry_msgs::Vector3Stamped& velocity)
{
    velocity_callback_timer.reset(callback_timeout);
    g_vel = velocity;
    if (debug)
    {
        printf( "frame_id: %s stamp: %d\n", g_vel.header.frame_id.c_str(), g_vel.header.stamp.sec );
        printf( "velocity: [%f %f %f]\n", g_vel.vector.x, g_vel.vector.y, g_vel.vector.z );
    }
}

void UAV::ultrasonic_callback(const sensor_msgs::LaserScan& scan)
{
    ultrasonic_callback_timer.reset(callback_timeout);
    g_scan = scan;
    if (debug)
    {
        printf( "frame_id: %s stamp: %d\n", g_scan.header.frame_id.c_str(), g_scan.header.stamp.sec );
        for (int i = 0; i < 5; i++)
            printf( "ultrasonic distance: [%f]  reliability: [%d]\n", g_scan.ranges[i], (int)g_scan.intensities[i] );
    }
}

void UAV::vision_callback(const geometry_msgs::Vector3Stamped& position)
{
    vision_callback_timer.reset(callback_timeout);
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

  float org_x = g_pos_calied.vector.x;
  float org_y = g_pos_calied.vector.y;
  float org_z = g_pos_calied.vector.z;

  float dis_x = dst_x - org_x;
  float dis_y = dst_y - org_y;
  float dis_z = dst_z - org_z; 

  float det_x, det_y, det_z;

  DJI::onboardSDK::FlightData flight_ctrl_data;
  flight_ctrl_data.flag = 0x90;
  flight_ctrl_data.z = dst_z;
  flight_ctrl_data.yaw = 0;

  int x_progress = 0; 
  int y_progress = 0; 
  int z_progress = 0; 
  while (x_progress < 100 || y_progress < 100 || z_progress <100) {

     flight_ctrl_data.x = dst_x - g_pos_calied.vector.x;
     flight_ctrl_data.y = dst_y - g_pos_calied.vector.y;
     control(flight_ctrl_data.flag, flight_ctrl_data.x, flight_ctrl_data.y, flight_ctrl_data.z, flight_ctrl_data.yaw);

     det_x = (100 * (dst_x - g_pos_calied.vector.x)) / dis_x;
     det_y = (100 * (dst_y - g_pos_calied.vector.y)) / dis_y;
     det_z = (100 * (dst_z - g_pos_calied.vector.z)) / dis_z;

     x_progress = 100 - (int)det_x;
     y_progress = 100 - (int)det_y;
     z_progress = 100 - (int)det_z;

     //lazy evaluation
     if (std::abs(dst_x - g_pos_calied.vector.x) < 0.1) x_progress = 100;
     if (std::abs(dst_y - g_pos_calied.vector.y) < 0.1) y_progress = 100;
     if (std::abs(dst_z - g_pos_calied.vector.z) < 0.1) z_progress = 100;

     guidance_nav_feedback.x_progress = x_progress;
     guidance_nav_feedback.y_progress = y_progress;
     guidance_nav_feedback.z_progress = z_progress;

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

    ref = g_pos_calied.vector.x + x;
    fdb = g_pos_calied.vector.x;
    PID_Calc(&pid[0], ref, fdb);
    ref = pid[0].out;
    fdb = g_vel.vector.x;
    PID_Calc(&pid[1], ref, fdb);

    ref = g_pos_calied.vector.y + y;
    fdb = g_pos_calied.vector.y;
    PID_Calc(&pid[2], ref, fdb);
    ref = pid[2].out;
    fdb = g_vel.vector.y;
    PID_Calc(&pid[3], ref, fdb);

    ref = g_pos_calied.vector.z + z;
    fdb = g_pos_calied.vector.z;
    PID_Calc(&pid[4], ref, fdb);
    ref = pid[4].out;
    fdb = g_vel.vector.z;
    PID_Calc(&pid[5], ref, fdb);
    
    /*
    float g_yaw = tf::getYaw(g_)
    ref = g_pos_calied.vector.yaw + yaw;
    fdb = g_pos_calied.vector.yaw;
    PID_Calc(&pid[6], ref, fdb);
    ref = pid[6].out;
    fdb = g_vel.vector.yaw;
    PID_Calc(&pid[7], ref, fdb);
    */

    float x_thr = pid[1].out;
    float y_thr = pid[3].out;
    float z_thr = pid[5].out;
    float yaw_thr = pid[7].out;

    uint8_t cflag = Flight::HorizontalLogic::HORIZONTAL_POSITION |
                        Flight::VerticalLogic::VERTICAL_VELOCITY |
                        Flight::YawLogic::YAW_ANGLE |
                        Flight::HorizontalCoordinate::HORIZONTAL_BODY |
                        Flight::SmoothMode::SMOOTH_ENABLE;

    bool result = control(cflag, x_thr, y_thr, z_thr, yaw_thr);

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
    return uart_write(uart_fd, buf, BUF_LEN) == BUF_LEN;
}

uint8_t UAV::stat_claw()
{
     #define BUF_LEN 4
    uint8_t buf[BUF_LEN];
    /*
    if (uart_read(uart_fd, buf, BUF_LEN) != BUF_LEN)
    {
        return 0xff;
    }
    if (buf[0] == 0xa5 && buf[2] == 0xfe && CRC8Check(buf, BUF_LEN, 0xff))
    {
        return buf[1];
    }
    */
    std::deque<uint8_t> que(BUF_LEN);
    Timer timer(serial_timeout);
    while (!timer.timeout())
    {
        uint8_t tmp;
        int len = uart_read(uart_fd, &tmp, 1);
        if (len == 1)
        {
            que.push_front(tmp);
        }
        if (que.size() == BUF_LEN)
        {
            for (int i = 0; i < BUF_LEN; i++)
            {
                buf[i] = que[i];
            }
            if (buf[0] == 0xa5 && buf[2] == 0xfe && CRC8Check(buf, BUF_LEN, 0xff))
            {
                return buf[1];
            }
            else
            {
                que.pop_back();
            }
        }
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
    if (status() == 3 && g_pos_calied.vector.z < -takeoff_height)
    {
        return true;
    }
    takingoff();
    return false;
}

bool UAV::fly_to_car()
{
    float x_err = dropoint.x - g_pos_calied.vector.x;
    float y_err = dropoint.y - g_pos_calied.vector.y;
    // float z_err = dropoint.z - g_pos_calied.vector.z;
    float z_err = 0;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence)
    {
        return true;
    }

    pid_control(x_err, y_err, z_err);

    return false;
}

bool UAV::serve_car()
{
    float x_err = v_pos.vector.x * vision_pos_coeff;
    float y_err = v_pos.vector.y * vision_pos_coeff;
    float z_err = dropoint.z - g_pos_calied.vector.z;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence)
    {
        return true;
    }

    pid_control(x_err, y_err, z_err);

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
    // float x_err = dropoint.x - g_pos_calied.vector.x;
    // float y_err = dropoint.y - g_pos_calied.vector.y;
    // float z_err = dropoint.z - g_pos_calied.vector.z;
    float x_err = 0;
    float y_err = 0;
    float z_err = -takeoff_height - g_pos_calied.vector.z;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence)
    {
        return true;
    }

    pid_control(x_err, y_err, z_err);

    return false;
}

bool UAV::fly_back()
{
    float x_err = g_pos_calied.vector.x;
    float y_err = g_pos_calied.vector.y;
    float z_err = 0;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence)
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
    float z_err = -landing_height - g_pos_calied.vector.z;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence)
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

void UAV::stateMachine()
{
    if (stat_claw() != 0xff)
    {
        serial_comm_timer.reset(serial_timeout);
    }
    if (serial_comm_timer.timeout() || position_callback_timer.timeout() || 
        velocity_callback_timer.timeout() || ultrasonic_callback_timer.timeout() || 
        vision_callback_timer.timeout())
    {
        std::cout << "stateMachine: sensor connection error, uav was forced to stand by!" << std::endl;
        release_control();
        ws = STAND_BY;
        return;
    }
    switch (ws)
    {
        case GRABBING:
        if (grab())
        {
            if (request_control())
            {
                ws = TAKING_OFF;
            }
        }
        else
        {
            std::cout << "stateMachine: grabing " << std::endl;
        }
        break;
        case TAKING_OFF:
        if (takeoff())
        {
            ws = FLYING_TO_CAR;
        }
        else
        {
            std::cout << "stateMachine: taking off " << std::endl;
        }
        break;
        case FLYING_TO_CAR:
        if (fly_to_car())
        {
            ws = SERVING_CAR;
        }
        else
        {
            std::cout << "stateMachine: flying to car " << std::endl;
        }
        break;
        case SERVING_CAR:
        if (serve_car())
        {
            ws = AIR_DROPPING;
        }
        else
        {
            std::cout << "stateMachine: serving car " << std::endl;
        }
        break;
        case AIR_DROPPING:
        if (air_drop())
        {
            ws = ASCENDING;
        }
        else
        {
            std::cout << "stateMachine: air-droping " << std::endl;
        }
        break;
        case ASCENDING:
        if (ascend())
        {
            ws = FLYING_BACK;
        }
        else
        {
            std::cout << "stateMachine: ascending " << std::endl;
        }
        break;
        case FLYING_BACK:
        if (fly_back())
        {
            ws = SERVING_PARK;
        }
        else
        {
            std::cout << "stateMachine: flying back " << std::endl;
        }
        break;
        case SERVING_PARK:
        if (serve_park())
        {
            ws = LANDING;
        }
        else
        {
            std::cout << "stateMachine: serving park " << std::endl;
        }
        break;
        case LANDING:
        if (land())
        {
            release_control(); // release drone control privilege
            calied = false;
            ws = STAND_BY;
        }
        else
        {
            std::cout << "stateMachine: landing " << std::endl;
        }
        break;
        default:
        ws = STAND_BY;
        release_control();
        std::cout << "stateMachine: stand-by " << std::endl;
        break;
    }
}

void UAV::spin()
{
    ros::Rate rate(spin_rate);
    while (ros::ok())
    {
        ros::spinOnce();

        stateMachine();

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
