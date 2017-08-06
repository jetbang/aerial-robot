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

#include "jet.h"

Jet::Jet(ros::NodeHandle& nh) : drone(nh), uart_fd(-1), calied(false)
{
    this->nh = nh;

    nh.param<int>("spin_rate", spin_rate, 50);
    nh.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
    nh.param<int>("serial_baudrate", serial_baudrate, 115200);
    nh.param<float>("vision_pos_coeff", vision_pos_coeff, 0.001f);
    nh.param<float>("takeoff_height", takeoff_height, 1.0f);
    nh.param<float>("landing_height", landing_height, 0.3f);

    int ret = uart_open(&uart_fd, serial_port.c_str(), serial_baudrate, UART_OFLAG_WR);
    if (ret < 0) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , serial_port.c_str());
    }

    grabber_service_client = nh.serviceClient<std_srvs::Empty>("/grabber/cmd");
    odometry_subscriber = nh.subscribe("/guidance/odometry", 10, &Jet::odometry_callback);
    vision_subscriber = nh.subscribe("/vision/position", 10, &Jet::vision_callback);

    uav_state_pub = nh.advertise<std_msgs::UInt8>("/jet_state", 10);
    guidance_odom_calied_pub = nh.advertise<nav_msgs::Odometry>("/odom_calied", 10);

    load_pid_param();

}

Jet::~Jet()
{

}

void Jet::load_dropoint_param(ros::NodeHandle& nh)
{
    nh.param<double>("/dropoint/x", dropoint.x, 0.0);
    nh.param<double>("/dropoint/y", dropoint.y, 0.0);
    nh.param<double>("/dropoint/z", dropoint.z, 0.0);
}

void Jet::fill_pid_param(ros::NodeHandle& nh, int i, const char* axis)
{
    std::stringstream ss;
    ss << "/pid/" << axis << "/";
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

void Jet::load_pid_param(ros::NodeHandle& nh)
{
    fill_pid_param(0, "xy");
    fill_pid_param(1, "xy");
    fill_pid_param(2, "z");
    fill_pid_param(3, "yaw");
}

void Jet::odometry_callback(const nav_msgs::OdometryConstPtr& odometry)
{
    odom = *odometry;
    if (calied == false)
    {
        odom_bias = odom;
        calied = true;
    }
    calc_odom_calied();
    publish_odom_calied();
}

void Jet::calc_odom_calied()
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

void Jet::publish_odom_calied()
{
    odom_calied_pub.publish(odom_calied);
}

void Jet::vision_callback(const geometry_msgs::Vector3Stamped& position)
{
    vision_target_pos = position;
}

bool Jet::cmd_grabber(uint8_t c)
{
    uint8_t buf[4];
    buf[0] = 0xa5;
    buf[1] = c;
    buf[2] = 0xfe;
    CRC8Append(buf, 0, 0xff);
    return (write(uart_fd, buf, 4) == BUF_LEN);
}

uint8_t Jet::stat_grabber()
{
    uint8_t buf[4];
    memset(buf, 0, 4);
    read(uart_fd, buf, 4);
    if (buf[0] == 0xa5 && buf[2] == 0xfe && CRC8Check(buf, 4, 0xff))
    {
        return buf[1];
    }
    return 0xff;
}

bool Jet::control(uint8_t ground, float x, float y, float z, float yaw)
{
    uint8_t flag = Flight::HorizontalLogic::HORIZONTAL_VELOCITY |
                        Flight::VerticalLogic::VERTICAL_VELOCITY |
                        Flight::YawLogic::YAW_RATE |
                        Flight::SmoothMode::SMOOTH_ENABLE;
    if (ground)
    {
        // WORLD
        flag |= Flight::HorizontalCoordinate::HORIZONTAL_GROUND;
    } else {
        // BODY
        flag |= Flight::HorizontalCoordinate::HORIZONTAL_BODY;
    }

    return drone.attitude_control(flag, x, y, z, yaw);
}

bool Jet::pid_control(uint8_t ground, float x, float y, float z, float yaw)
{
    // x
    float fx = odom_calied.pose.pose.position.x;
    float rx = odom_calied.pose.pose.position.x + x;
    PID_Calc(&pid[0], rx, fx);

    // y
    float fy = odom_calied.pose.pose.position.y;
    float ry = odom_calied.pose.pose.position.y + y;
    PID_Calc(&pid[1], ry, fy);

    // z
    float fz = odom_calied.pose.pose.position.z;
    float rz = odom_calied.pose.pose.position.z + z;
    PID_Calc(&pid[2], rz, fz);
    
    // yaw
    float fyaw = tf::getYaw(odom_calied.pose.pose.orientation);
    float ryaw = fyaw + yaw;
    PID_Calc(&pid[3], ryaw, fyaw);

    float ox = pid[0].out;
    float oy = pid[1].out;
    float oz = pid[2].out;
    float oyaw = pid[3].out;

    control(ground, ox, oy, oz, oyaw);

    for (int i = 0; i < 4; i++)
    {
        if (pid[i].out != 0)
            return false;
    }

    return true;
}

uint8_t Jet::status()
{
    return drone.flight_status;
}

bool Jet::doGrabBullets()
{
    return cmd_grabber(1);
}

bool Jet::doRequestControl()
{
    return drone.request_sdk_permission_control();
}

bool Jet::doTakeoff()
{
    return drone.takeoff();
}

bool Jet::doFlyToCar()
{
    float ex = dropoint.x - odom_calied.pose.pose.position.x;
    float ey = dropoint.y - odom_calied.pose.pose.position.y;
    float ez = 0;
    float eyaw = 0;

    pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doServeCar()
{
    float ex = vision_target_pos.vector.x * vision_pos_coeff;
    float ey = vision_target_pos.vector.y * vision_pos_coeff;
    float ez = dropoint.z - odom_calied.pose.pose.position.z;
    float eyaw = 0;

    pid_control(0, ex, ey, ez, eyaw); // Body frame
}

bool Jet::doDropBullets()
{
    return cmd_grabber(0);
}

bool Jet::doBackToNormalAltitude()
{
    float ex = 0;
    float ey = 0;
    float ez = -takeoff_height - odom_calied.pose.pose.position.z; // normal altitude
    float eyaw = 0;

    pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doFlyBack()
{
    float ex = 0 - odom_calied.pose.pose.position.x;
    float ey = 0 - odom_calied.pose.pose.position.y;
    float ez = 0;
    float eyaw = 0;

    pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doVisualServoLanding()
{
    float ex = vision_target_pos.vector.x * vision_pos_coeff;
    float ey = vision_target_pos.vector.y * vision_pos_coeff;
    float ez = -landing_height - odom_calied.pose.pose.position.z; // normal altitude
    float eyaw = 0;

    pid_control(0, ex, ey, ez, eyaw); // Body frame
}

bool Jet::doLanding()
{
    return drone.landing();
}

bool Jet::action(uint8_t cmd)
{
    switch (cmd)
    {
        case GRAB_BULLETS:
        return doGrabBullets();

        case REQUEST_CONTROL:
        return doRequestControl();

        case TAKE_OFF:
        return doTakeoff();

        case FLY_TO_CAR:
        return doFlyToCar();

        case SERVE_CAR:
        return doServeCar();

        case DROP_BULLETS:
        return doDropBullets();

        case BACK_TO_NORMAL_ALTITUDE:
        return doBackToNormalAltitude();

        case FLY_BACK:
        return doFlyBack();

        case VISUAL_SERVO_LANDING:
        return doVisualServoLanding();

        case LANDING:
        return doLanding();
    }
}

void Jet::stateMachine()
{
}

void Jet::spin()
{
    ros::Rate rate(spin_rate);

    while (ros::ok())
    {
        ros::spinOnce();

        char c = kbhit;

        rate.sleep();
    }
}