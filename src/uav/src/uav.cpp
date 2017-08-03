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

UAV::UAV() : nh("~"), debug(true), ws(STAND_BY), xy_err_tolerence(0.1), z_err_tolerence(0.1),
    uart_fd(-1), serial_port("/dev/ttyTHS2"), serial_baudrate(115200), spin_rate(50), 
    open_claw_cmd(0), close_claw_cmd(1)
{
    ros::NodeHandle np("~");
    np.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
    np.param<int>("serial_baudrate", serial_baudrate, 115200);
    np.param<int>("open_claw_cmd", open_claw_cmd, 0);
    np.param<int>("close_claw_cmd", close_claw_cmd, 1);
    np.param<int>("spin_rate", spin_rate, 50);
    np.param<float>("xy_err_tolerence", xy_err_tolerence, 0.1f);
    np.param<float>("z_err_tolerence", z_err_tolerence, 0.1f);

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
    position_sub = nh.subscribe("/guidance/position", 1, &UAV::position_callback, this);
    velocity_sub = nh.subscribe("/guidance/velocity", 1, &UAV::velocity_callback, this);
    ultrasonic_sub = nh.subscribe("/guidance/ultrasonic", 1, &UAV::ultrasonic_callback, this);
    vision_sub = nh.subscribe("/vision/position", 1, &UAV::vision_callback, this);

    // initialize services
    charge_service  = nh.advertiseService("charge", &UAV::charge_service_callback, this);

    // initialize dji drone
    drone = new DJIDrone(nh);

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
    }
}

void UAV::position_callback(const geometry_msgs::Vector3Stamped& position)
{
    g_pos = position;
    if (debug)
    {
        printf("frame_id: %s stamp: %d\n", g_pos.header.frame_id.c_str(), g_pos.header.stamp.sec);
        for (int i = 0; i < 5; i++)
            printf("global position: [%f %f %f]\n", g_pos.vector.x, g_pos.vector.y, g_pos.vector.z);
    }
}

void UAV::velocity_callback(const geometry_msgs::Vector3Stamped& velocity)
{
    g_vel = velocity;
    if (debug)
    {
        printf( "frame_id: %s stamp: %d\n", g_vel.header.frame_id.c_str(), g_vel.header.stamp.sec );
        printf( "velocity: [%f %f %f]\n", g_vel.vector.x, g_vel.vector.y, g_vel.vector.z );
    }
}

void UAV::ultrasonic_callback(const sensor_msgs::LaserScan& scan)
{
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
    v_pos = position;
    if (debug)
    {
        printf("frame_id: %s stamp: %d\n", v_pos.header.frame_id.c_str(), v_pos.header.stamp.sec);
        for (int i = 0; i < 5; i++)
            printf("vision position: [%f %f %f]\n", v_pos.vector.x, v_pos.vector.y, v_pos.vector.z);
    }
}

bool UAV::charge_service_callback(uav::Charge::Request& request, uav::Charge::Response& response)
{
    if (ws == STAND_BY)
    {
        ws = GRABBING;
    }
    response.result = ws;
    return true;
}

bool UAV::attach()
{
    bool result = drone->request_sdk_permission_control();
    if (debug)
    {
        printf( "UAV::attach result: %d\n", result);
    }
    return result;
}

bool UAV::detach()
{
    bool result = drone->release_sdk_permission_control();
    if (debug)
    {
        printf( "UAV::detach result: %d\n", result);
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

bool UAV::reduce_err(float x, float y, float z, float yaw = 0)
{
    float ref = 0;
    float fdb = 0;

    ref = g_pos.vector.x + x;
    fdb = g_pos.vector.x;
    PID_Calc(&pid[0], ref, fdb);
    ref = pid[0].out;
    fdb = g_vel.vector.x;
    PID_Calc(&pid[1], ref, fdb);

    ref = g_pos.vector.y + y;
    fdb = g_pos.vector.y;
    PID_Calc(&pid[2], ref, fdb);
    ref = pid[2].out;
    fdb = g_vel.vector.y;
    PID_Calc(&pid[3], ref, fdb);

    ref = g_pos.vector.z + z;
    fdb = g_pos.vector.z;
    PID_Calc(&pid[4], ref, fdb);
    ref = pid[4].out;
    fdb = g_vel.vector.z;
    PID_Calc(&pid[5], ref, fdb);
    
    /*
    float g_yaw = tf::getYaw(g_)
    ref = g_pos.vector.yaw + yaw;
    fdb = g_pos.vector.yaw;
    PID_Calc(&pid[6], ref, fdb);
    ref = pid[6].out;
    fdb = g_vel.vector.yaw;
    PID_Calc(&pid[7], ref, fdb);
    */

    float x_thr = pid[1].out;
    float y_thr = pid[3].out;
    float z_thr = pid[5].out;
    float yaw_thr = pid[7].out;

    bool result = control(0x40, x_thr, y_thr, z_thr, yaw_thr);

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

bool UAV::open_claw()
{
    return cmd_claw(open_claw_cmd);
}

bool UAV::close_claw()
{
    return cmd_claw(close_claw_cmd);
}

bool UAV::grab()
{
    close_claw();
    return true;
}

bool UAV::takeoff()
{
    takingoff();
    return true;
}

bool UAV::fly_to_car()
{
    float x_err = v_pos.vector.x;
    float y_err = v_pos.vector.y;
    float z_err = g_pos.vector.z;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence)
    {
        return true;
    }

    reduce_err(x_err, y_err, z_err);

    return false;
}

bool UAV::serve_car()
{
    float x_err = v_pos.vector.x;
    float y_err = v_pos.vector.y;
    float z_err = g_pos.vector.z;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence)
    {
        return true;
    }

    reduce_err(x_err, y_err, z_err);

    return false;
}

bool UAV::air_drop()
{
    open_claw();
    return true;
}

bool UAV::ascend()
{
    float x_err = v_pos.vector.x;
    float y_err = v_pos.vector.y;
    float z_err = g_pos.vector.z;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence)
    {
        return true;
    }

    reduce_err(x_err, y_err, z_err);

    return false;
}

bool UAV::fly_back()
{
    float x_err = v_pos.vector.x;
    float y_err = v_pos.vector.y;
    float z_err = g_pos.vector.z;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence)
    {
        return true;
    }

    reduce_err(x_err, y_err, z_err);

    return false;
}

bool UAV::serve_park()
{
    float x_err = v_pos.vector.x;
    float y_err = v_pos.vector.y;
    float z_err = g_pos.vector.z;

    if (x_err < xy_err_tolerence && y_err < xy_err_tolerence && z_err < z_err_tolerence)
    {
        return true;
    }

    reduce_err(x_err, y_err, z_err);

    return false;
}

bool UAV::land()
{

}

void UAV::stateMachine()
{
    switch (ws)
    {
        break;
        case GRABBING:
        if (grab())
        {
            ws = TAKING_OFF;
        }
        break;
        case TAKING_OFF:
        if (attach() && takeoff())
        {
            ws = FLYING_TO_CAR;
        }
        break;
        case FLYING_TO_CAR:
        if (fly_to_car())
        {
            ws = SERVING_CAR;
        }
        break;
        case SERVING_CAR:
        if (serve_car())
        {
            ws = AIR_DROPPING;
        }
        break;
        case AIR_DROPPING:
        if (air_drop())
        {
            ws = ASCENDING;
        }
        break;
        case ASCENDING:
        if (ascend())
        {
            ws = FLYING_BACK;
        }
        break;
        case FLYING_BACK:
        if (fly_back())
        {
            ws = SERVING_PARK;
        }
        break;
        case SERVING_PARK:
        if (serve_park())
        {
            ws = LANDING;
        }
        break;
        case LANDING:
        if (land())
        {
            ws = STAND_BY;
        }
        break;
        default:
        ws = STAND_BY;
        break;
    }
}

void UAV::spin()
{
    ros::Rate rate(50);
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
