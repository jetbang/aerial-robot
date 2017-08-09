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

Jet::Jet(ros::NodeHandle& nh) : drone(nh), uart_fd(-1), calied(false), odom_update_flag(false),
vision_target_pos_update_flag(false), use_guidance(false), freestyle(false), 
odom_callback_timer(60), vision_callback_timer(500)
{
    this->nh = nh;

    ros::NodeHandle np("~");

    np.param<int>("spin_rate", spin_rate, 50);
    np.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
    np.param<int>("serial_baudrate", serial_baudrate, 115200);
    np.param<bool>("use_guidance", use_guidance, false);

    int ret = uart_open(&uart_fd, serial_port.c_str(), serial_baudrate, UART_OFLAG_WR);
    if (ret < 0) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , serial_port.c_str());
    }

    load_pid_param(nh);
    load_vision_param(nh);
    load_flight_param(nh);
    load_dropoint_param(nh);
    load_duration_param(nh);
    load_timeout_param(nh);

    charge_srv = nh.advertiseService("/charge", &Jet::charge_callback, this);
    cmd_grabber_srv = nh.advertiseService("/grabber/cmd", &Jet::cmd_grabber_callback, this);
    stat_grabber_srv = nh.advertiseService("/grabber/stat", &Jet::stat_grabber_callback, this);

    reload_pid_param_srv = nh.advertiseService("/reload_pid_param", &Jet::reload_pid_param_callback, this);
    reload_vision_param_srv = nh.advertiseService("/reload_vision_param", &Jet::reload_vision_param_callback, this);
    reload_flight_param_srv = nh.advertiseService("/reload_flight_param", &Jet::reload_flight_param_callback, this);
    reload_dropoint_param_srv = nh.advertiseService("/reload_dropoint_param", &Jet::reload_dropoint_param_callback, this);
    reload_duration_param_srv = nh.advertiseService("/reload_duration_param", &Jet::reload_duration_param_callback, this);

    odometry_sub = nh.subscribe("/odom_raw", 10, &Jet::odometry_callback, this);
    vision_sub = nh.subscribe("/vision/target_pos", 10, &Jet::vision_callback, this);

    jet_state_pub = nh.advertise<std_msgs::UInt8>("/jet_state", 10);
    odom_calied_pub = nh.advertise<nav_msgs::Odometry>("/odom_calied", 10);

    jet_nav_action_server = new JetNavActionServer(nh,
            "jet_nav_action",
            boost::bind(&Jet::jet_nav_action_callback, this, _1), false);
    jet_nav_action_server->start();

}

Jet::~Jet()
{
    delete jet_nav_action_server;
    if (uart_fd != -1)
    {
        close(uart_fd);
        uart_fd = -1;
    }
}

void Jet::load_dropoint_param(ros::NodeHandle& nh)
{
    nh.param<double>("/dropoint/x", dropoint.x, 0.0);
    nh.param<double>("/dropoint/y", dropoint.y, 0.0);
    nh.param<double>("/dropoint/z", dropoint.z, 0.0);

    std::cout << "dropoint: [" << dropoint.x << ", " 
              << dropoint.y << ", " << dropoint.z << "]" << std::endl;
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

    std::cout << "pid " << axis << " : { kp: " << pid[i].kp 
              << ", ki: " << pid[i].ki << ", kd: "<< pid[i].kd
              << ", db: " << pid[i].db << ", it: "<< pid[i].it
              << ", Emax: " << pid[i].Emax << ", Pmax: "<< pid[i].Pmax
              << ", Imax: " << pid[i].Imax << ", Dmax: "<< pid[i].Dmax
              << ", Omax: " << pid[i].Omax << "}" << std::endl;
}

void Jet::load_pid_param(ros::NodeHandle& nh)
{
    fill_pid_param(nh, 0, "xy");
    fill_pid_param(nh, 1, "xy");
    fill_pid_param(nh, 2, "z");
    fill_pid_param(nh, 3, "yaw");
}

void Jet::load_vision_param(ros::NodeHandle& nh)
{
    nh.param<float>("vision_pos_coeff", vision_pos_coeff, 0.001f);
    std::cout << "vision_pos_coeff: " << vision_pos_coeff << std::endl;
}

void Jet::load_flight_param(ros::NodeHandle& nh)
{
    nh.param<float>("takeoff_height", takeoff_height, 1.2f);
    nh.param<float>("landing_height", landing_height, 0.3f);
    nh.param<float>("normal_altitude", normal_altitude, 1.2);
    std::cout << "takeoff_height: " << takeoff_height << std::endl;
    std::cout << "landing_height: " << landing_height << std::endl;
    std::cout << "normal_altitude: " << normal_altitude << std::endl;
}

void Jet::load_duration_param(ros::NodeHandle& nh)
{
    nh.getParam("duration", duration);
    std::cout << "durations: [" ;
    for (int i = 0; i < duration.size() - 1; i++)
    {
        std::cout << duration[i] << ", " ;
    }
    std::cout << duration[duration.size() - 1] << "]" << std::endl;
}

void Jet::load_timeout_param(ros::NodeHandle& nh)
{
    nh.param<int>("odom_callback_timeout", odom_callback_timeout, 60);
    nh.param<int>("vision_callback_timeout", vision_callback_timeout, 500);
    std::cout << "odom_callback_timeout: " << odom_callback_timeout << std::endl;
    std::cout << "vision_callback_timeout: " << vision_callback_timeout << std::endl;
}

void Jet::odometry_callback(const nav_msgs::OdometryConstPtr& odometry)
{
    odom_callback_timer.reset(odom_callback_timeout);
    odom = *odometry;
    if (use_guidance) // revert z if use guidance
    {
        odom.pose.pose.position.z = -odom.pose.pose.position.z;
    }
    if (calied == false)
    {
        odom_bias = odom;
        calied = true;
    }
    calc_odom_calied();
    pub_odom_calied();
    odom_update_flag = true;
}

void Jet::calc_odom_calied()
{
    jet_yaw = tf::getYaw(odom.pose.pose.orientation);
    jet_yaw_bias = tf::getYaw(odom_bias.pose.pose.orientation);
    jet_yaw_calied = jet_yaw - jet_yaw_bias;

    geometry_msgs::Quaternion quat_calied = tf::createQuaternionMsgFromYaw(jet_yaw_calied);

    odom_calied.header = odom.header;
    odom_calied.child_frame_id = odom.child_frame_id;
    odom_calied.pose.pose.position.x = odom.pose.pose.position.x - odom_bias.pose.pose.position.x;
    odom_calied.pose.pose.position.y = odom.pose.pose.position.y - odom_bias.pose.pose.position.y;
    odom_calied.pose.pose.position.z = odom.pose.pose.position.z - odom_bias.pose.pose.position.z;
    odom_calied.pose.pose.orientation = quat_calied;
    odom_calied.twist = odom.twist;
}

void Jet::pub_odom_calied()
{
    odom_calied_pub.publish(odom_calied);
}

void Jet::pub_jet_state()
{
    std_msgs::UInt8 msg;
    msg.data = jet_state;
    jet_state_pub.publish(msg);
}

void Jet::vision_callback(const geometry_msgs::PoseStamped& position)
{
    vision_callback_timer.reset(vision_callback_timeout);
    vision_target_relative_pos = position;

    // tf
    vision_target_global_pos.header = position.header;

    vision_target_relative_yaw = tf::getYaw(position.pose.orientation);
    vision_target_global_yaw = jet_yaw_calied + vision_target_relative_yaw;
    
    vision_target_global_pos.pose.position.x = odom_calied.pose.pose.position.x + vision_target_global_pos.pose.position.x;
    vision_target_global_pos.pose.position.y = odom_calied.pose.pose.position.y + vision_target_global_pos.pose.position.y;
    vision_target_global_pos.pose.position.z = odom_calied.pose.pose.position.z + vision_target_global_pos.pose.position.z;
    vision_target_global_pos.pose.orientation = tf::createQuaternionMsgFromYaw(vision_target_global_yaw);

    vision_target_pos_update_flag = true;
}

bool Jet::charge_callback(jet::Charge::Request& request, jet::Charge::Response& response)
{
    response.result = status();
    if (jet_state == STAND_BY) // Accept request ONLY when jet is standing by 
    {
        jet_state = GRAB_BULLETS;
    }
    return true;
}

bool Jet::cmd_grabber_callback(jet::CmdGrabber::Request& request, jet::CmdGrabber::Response& response)
{
    response.result = cmd_grabber(request.cmd);
    return true;
}

bool Jet::stat_grabber_callback(jet::StatGrabber::Request& request, jet::StatGrabber::Response& response)
{
    response.result = stat_grabber();
    return true;
}

bool Jet::reload_pid_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_pid_param(nh);
    return true;
}

bool Jet::reload_dropoint_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_dropoint_param(nh);
    return true;
}

bool Jet::reload_duration_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_duration_param(nh);
    return true;
}

bool Jet::reload_vision_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_vision_param(nh);
    return true;
}

bool Jet::reload_flight_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_flight_param(nh);
    return true;
}

bool Jet::reload_timeout_param_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    load_timeout_param(nh);
    return true;
}

bool Jet::cmd_grabber(uint8_t c)
{
    if (uart_fd == -1)
    {
        return false;
    }
    uint8_t buf[4];
    buf[0] = 0xa5;
    buf[1] = c;
    buf[2] = 0xfe;
    CRC8Append(buf, 4, 0xff);
    write(uart_fd, buf, 4);
    return true;
}

uint8_t Jet::stat_grabber()
{
    if (uart_fd == -1)
    {
        return 0xff;
    }

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

bool Jet::goal_reached()
{
    for (int i = 0; i < 4; i++)
    {
        if (pid[i].out != 0)
            return false;
    }
    return true;
}

bool Jet::pid_control(uint8_t ground, float x, float y, float z, float yaw)
{
    float fx = 0;
    float fy = 0;
    float fz = 0;
    float fyaw = 0;

    float rx = 0;
    float ry = 0;
    float rz = 0;
    float ryaw = 0;

    if (ground)
    {
        fx = odom_calied.pose.pose.position.x;
        fy = odom_calied.pose.pose.position.y;
        fz = odom_calied.pose.pose.position.z;
        fyaw = tf::getYaw(odom_calied.pose.pose.orientation);
    }
    else
    {
        fx = 0;
        fy = 0;
        fz = 0;
        fyaw = 0;
    }
    
    rx = fx + x;
    ry = fy + y;
    rz = fz + z;
    ryaw = fyaw + yaw;

    // x
    PID_Calc(&pid[0], rx, fx);

    // y
    PID_Calc(&pid[1], ry, fy);

    // z
    PID_Calc(&pid[2], rz, fz);
    
    // yaw
    PID_Calc(&pid[3], ryaw, fyaw);

    float ox = pid[0].out;
    float oy = pid[1].out;
    float oz = pid[2].out;
    float oyaw = pid[3].out;

    control(ground, ox, oy, oz, oyaw);

    std::cout << "+------------------------- control loop -------------------------+" << std::endl;
    std::cout << "rx: " << rx << ", ry: " << ry << ", rz: " << rz << ", ryaw: " << ryaw << std::endl;
    std::cout << "fx: " << fx << ", fy: " << fy << ", fz: " << fz << ", fyaw: " << fyaw << std::endl;
    std::cout << "ex: " << x << ", ey: " << y << ", ez: " << z << ", eyaw: " << yaw << std::endl;
    std::cout << "ox: " << ox << ", oy: " << oy << ", oz: " << oz << ", oyaw: " << oyaw << std::endl;
    std::cout << "+----------------------------------------------------------------+" << std::endl;
    return goal_reached();
}

bool Jet::jet_nav_action_callback(const jet::JetNavGoalConstPtr& goal)
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

    int x_progress = 0; 
    int y_progress = 0; 
    int z_progress = 0; 
    int yaw_progress = 0;

    while (x_progress < 100 || y_progress < 100 || z_progress < 100 || yaw_progress < 100) {

        float ex = dst_x - odom_calied.pose.pose.position.x;
        float ey = dst_y - odom_calied.pose.pose.position.y;
        float ez = dst_z - odom_calied.pose.pose.position.z;
        float eyaw = dst_yaw - tf::getYaw(odom_calied.pose.pose.orientation);

        pid_control(1, ex, ey, ez, eyaw);

        float det_x = (100 * ex) / dis_x;
        float det_y = (100 * ey) / dis_y;
        float det_z = (100 * ez) / dis_z;
        float det_yaw = (100 * eyaw) / dis_yaw;

        x_progress = 100 - (int)det_x;
        y_progress = 100 - (int)det_y;
        z_progress = 100 - (int)det_z;
        yaw_progress = 100 - (int)det_yaw;

        //lazy evaluation
        if (pid[0].out == 0) x_progress = 100;
        if (pid[1].out == 0) y_progress = 100;
        if (pid[2].out == 0) z_progress = 100;
        if (pid[3].out == 0) yaw_progress = 100;

        jet_nav_feedback.x_progress = x_progress;
        jet_nav_feedback.y_progress = y_progress;
        jet_nav_feedback.z_progress = z_progress;
        jet_nav_feedback.yaw_progress = yaw_progress;

        jet_nav_action_server->publishFeedback(jet_nav_feedback);

        usleep(20000); // 50hz
    }

    jet_nav_result.result = true;
    jet_nav_action_server->setSucceeded(jet_nav_result);

    return true;
}

uint8_t Jet::status()
{
    return drone.flight_status;
}

bool Jet::doStandby()
{
    return true;
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

bool Jet::doToNormalAltitude()
{
    float ex = 0;
    float ey = 0;
    float ez = normal_altitude - odom_calied.pose.pose.position.z; // adjust altitude
    float eyaw = 0;

    return pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doFlyToCar()
{
    float ex = dropoint.x - odom_calied.pose.pose.position.x;
    float ey = dropoint.y - odom_calied.pose.pose.position.y;
    float ez = 0;
    float eyaw = 0;

    return pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doFindCar()
{
    if (vision_target_pos_update_flag)
    {
        vision_target_pos_update_flag = false;
        return true;
    }
    return false;
}

bool Jet::doServeCar()
{
    float ex = vision_target_global_pos.pose.position.x - odom_calied.pose.pose.position.x;
    float ey = vision_target_global_pos.pose.position.y - odom_calied.pose.pose.position.y;
    float ez = dropoint.z + vision_target_global_pos.pose.position.z - odom_calied.pose.pose.position.z;
    float eyaw = 0;

    return pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doDropBullets()
{
    return cmd_grabber(0);
}

bool Jet::doBackToNormalAltitude()
{
    float ex = 0;
    float ey = 0;
    float ez = normal_altitude - odom_calied.pose.pose.position.z; // normal altitude
    float eyaw = 0;

    return pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doFlyBack()
{
    float ex = 0 - odom_calied.pose.pose.position.x;
    float ey = 0 - odom_calied.pose.pose.position.y;
    float ez = 0;
    float eyaw = 0;

    return pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doFindPark()
{
    if (vision_target_pos_update_flag)
    {
        vision_target_pos_update_flag = false;
        return true;
    }
    return false;
}

bool Jet::doVisualServoLanding()
{
    float ex = vision_target_global_pos.pose.position.x - odom_calied.pose.pose.position.x;
    float ey = vision_target_global_pos.pose.position.y - odom_calied.pose.pose.position.y;
    float ez = landing_height + vision_target_global_pos.pose.position.z - odom_calied.pose.pose.position.z;
    float eyaw = 0;

    return pid_control(1, ex, ey, ez, eyaw);
}

bool Jet::doLanding()
{
    return drone.landing();
}

bool Jet::doReleaseControl()
{
    return drone.release_sdk_permission_control();
}

bool Jet::cmd_jet_state(uint8_t cmd)
{
    if (cmd <= RELEASE_CONTROL)
    {
        jet_state = cmd;
    }
}

bool Jet::action(uint8_t cmd)
{
    cmd_jet_state(cmd);
    switch (cmd)
    {
        case STAND_BY:
        std::cout << "\nAction: " << "Standby" << std::endl;
        return doStandby();

        case GRAB_BULLETS:
        std::cout << "\nAction: " << "Grab Bullets" << std::endl;
        return doGrabBullets();

        case REQUEST_CONTROL:
        std::cout << "\nAction: " << "Request Control" << std::endl;
        return doRequestControl();

        case TAKE_OFF:
        std::cout << "\nAction: " << "Takeoff" << std::endl;
        return doTakeoff();

        case TO_NORMAL_ALTITUDE:
        std::cout << "\nAction: " << "To Normal Altitude" << std::endl;
        return doToNormalAltitude();

        case FLY_TO_CAR:
        std::cout << "\nAction: " << "Fly to Car" << std::endl;
        return doFlyToCar();

        case FIND_CAR:
        std::cout << "\nAction: " << "Find Car" << std::endl;
        return doFindCar();

        case SERVE_CAR:
        std::cout << "\nAction: " << "Serve Car" << std::endl;
        return doServeCar();

        case DROP_BULLETS:
        std::cout << "\nAction: " << "Drop Bullets" << std::endl;
        return doDropBullets();

        case BACK_TO_NORMAL_ALTITUDE:
        std::cout << "\nAction: " << "Back to Normal Altitude" << std::endl;
        return doBackToNormalAltitude();

        case FLY_BACK:
        std::cout << "\nAction: " << "Fly Back" << std::endl;
        return doFlyBack();

        case FIND_PARK:
        std::cout << "\nAction: " << "Find Park" << std::endl;
        return doFindPark();

        case VISUAL_SERVO_LANDING:
        std::cout << "\nAction: " << "Visual Servo Landing" << std::endl;
        return doVisualServoLanding();

        case LANDING:
        std::cout << "\nAction: " << "Landing" << std::endl;
        return doLanding();

        case RELEASE_CONTROL:
        std::cout << "\nAction: " << "Release Control" << std::endl;
        return doReleaseControl();

        default:
        return false;
    }
}

void Jet::stateMachine()
{
    static bool success = false;
    static uint32_t tick = 0;
    switch (jet_state)
    {
        case STAND_BY:
        if (!success)
        {
            success = doStandby();
            std::cout << "stateMachine: " << "Standby" << std::endl;
        }
        else if (tick < duration[STAND_BY])
        {
            tick++;
            std::cout << "stateMachine: " << "Standby@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = GRAB_BULLETS;
            std::cout << "stateMachine: " << "Standby->Grab Bullets" << tick << std::endl;
        }
        break;

        case GRAB_BULLETS:
        if (!success)
        {
            success = doGrabBullets();
            std::cout << "stateMachine: " << "Grab Bullets" << std::endl;
        }
        else if (tick < duration[GRAB_BULLETS])
        {
            tick++;
            std::cout << "stateMachine: " << "Grab Bullets@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = REQUEST_CONTROL;
            std::cout << "stateMachine: " << "Grab Bullets->Request Control" << tick << std::endl;
        }
        break;

        case REQUEST_CONTROL:
        if (!success)
        {
            success = doRequestControl();
            std::cout << "stateMachine: " << "Request Control" << std::endl;
        }
        else if (tick < duration[REQUEST_CONTROL])
        {
            tick++;
            std::cout << "stateMachine: " << "Request Control@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = TAKE_OFF;
            std::cout << "stateMachine: " << "Request Control->Takeoff" << tick << std::endl;
        }
        break;

        case TAKE_OFF:
        if (!success)
        {
            success = doTakeoff();
            std::cout << "stateMachine: " << "Takeoff" << std::endl;
        }
        else if (tick < duration[TAKE_OFF])
        {
            tick++;
            std::cout << "stateMachine: " << "Takeoff@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = TO_NORMAL_ALTITUDE;
            std::cout << "stateMachine: " << "Takeoff->To Normal Altitude" << tick << std::endl;
        }
        break;

        case TO_NORMAL_ALTITUDE:
        if (!success)
        {
            success = doToNormalAltitude();
            std::cout << "stateMachine: " << "To Normal Altitude" << std::endl;
        }
        else if (tick < duration[TO_NORMAL_ALTITUDE])
        {
            tick++;
            std::cout << "stateMachine: " << "To Normal Altitude@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = FLY_TO_CAR;
            std::cout << "stateMachine: " << "To Normal Altitude->Fly to Car" << tick << std::endl;
        }
        break;

        case FLY_TO_CAR:
        if (!success)
        {
            success = doFlyToCar();
            std::cout << "stateMachine: " << "Fly to Car" << std::endl;
        }
        else if (tick < duration[FLY_TO_CAR])
        {
            tick++;
            std::cout << "stateMachine: " << "Fly to Car@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = FIND_CAR;
            std::cout << "stateMachine: " << "Fly to Car->Find Car" << tick << std::endl;
        }
        break;

        case FIND_CAR:
        if (!success)
        {
            success = doFindCar();
            std::cout << "stateMachine: " << "Find Car" << std::endl;
        }
        else if (tick < duration[FIND_CAR])
        {
            tick++;
            std::cout << "stateMachine: " << "Find Car@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = SERVE_CAR;
            std::cout << "stateMachine: " << "Find Car->Serve Car" << tick << std::endl;
        }
        break;

        case SERVE_CAR:
        if (!success)
        {
            success = doServeCar();
            std::cout << "stateMachine: " << "Serve Car" << std::endl;
        }
        else if (tick < duration[SERVE_CAR])
        {
            tick++;
            std::cout << "stateMachine: " << "Serve Car@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = DROP_BULLETS;
            std::cout << "stateMachine: " << "Serve Car->Drop Bullets" << tick << std::endl;
        }
        break;

        case DROP_BULLETS:
        if (!success)
        {
            success = doDropBullets();
            std::cout << "stateMachine: " << "Drop Bullets" << std::endl;
        }
        else if (tick < duration[DROP_BULLETS])
        {
            tick++;
            std::cout << "stateMachine: " << "Drop Bullets@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = BACK_TO_NORMAL_ALTITUDE;
            std::cout << "stateMachine: " << "Drop Bullets->Back to Normal Altitude" << tick << std::endl;
        }
        break;

        case BACK_TO_NORMAL_ALTITUDE:
        if (!success)
        {
            success = doBackToNormalAltitude();
            std::cout << "stateMachine: " << "Back to Normal Altitude" << std::endl;
        }
        else if (tick < duration[BACK_TO_NORMAL_ALTITUDE])
        {
            tick++;
            std::cout << "stateMachine: " << "Back to Normal Altitude@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = FLY_BACK;
            std::cout << "stateMachine: " << "Back to Normal Altitude->Fly Back" << tick << std::endl;
        }
        break;

        case FLY_BACK:
        if (!success)
        {
            success = doFlyBack();
            std::cout << "stateMachine: " << "Fly Back" << std::endl;
        }
        else if (tick < duration[FLY_BACK])
        {
            tick++;
            std::cout << "stateMachine: " << "Fly Back@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = FIND_PARK;
            std::cout << "stateMachine: " << "Fly Back->Find Park" << tick << std::endl;
        }
        break;

        case FIND_PARK:
        if (!success)
        {
            success = doFindPark();
            std::cout << "stateMachine: " << "Find Park" << std::endl;
        }
        else if (tick < duration[FIND_PARK])
        {
            tick++;
            std::cout << "stateMachine: " << "Find Park@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = VISUAL_SERVO_LANDING;
            std::cout << "stateMachine: " << "Find Park->Visual Servo Landing" << tick << std::endl;
        }
        break;

        case VISUAL_SERVO_LANDING:
        if (!success)
        {
            success = doVisualServoLanding();
            std::cout << "stateMachine: " << "Visual Servo Landing" << std::endl;
        }
        else if (tick < duration[VISUAL_SERVO_LANDING])
        {
            tick++;
            std::cout << "stateMachine: " << "Visual Servo Landing@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = LANDING;
            std::cout << "stateMachine: " << "Visual Servo Landing->Landing" << tick << std::endl;
        }
        break;

        case LANDING:
        if (!success)
        {
            success = doLanding();
            std::cout << "stateMachine: " << "Landing" << std::endl;
        }
        else if (tick < duration[LANDING])
        {
            tick++;
            std::cout << "stateMachine: " << "Landing@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = RELEASE_CONTROL;
            std::cout << "stateMachine: " << "Landing->Standby" << tick << std::endl;

            calied = false; // re-calibrate odom
        }
        break;

        case RELEASE_CONTROL:
        if (!success)
        {
            success = doReleaseControl();
            std::cout << "stateMachine: " << "Release Control" << std::endl;
        }
        else if (tick < duration[RELEASE_CONTROL])
        {
            tick++;
            std::cout << "stateMachine: " << "Release Control@Tick: " << tick << std::endl;
        }
        else
        {
            tick = 0;
            success = false;
            jet_state = STAND_BY;
            std::cout << "stateMachine: " << "Release Control->Standby" << tick << std::endl;
            if (freestyle)
            {
                freestyle = false; // clear freestyle flag
                std::cout << "+---------------------Jetbang Free Style Done----------------------+" << std::endl;
                help();
            }
        }
        break;

        default:
        jet_state = STAND_BY;
        break;
    }
}

void Jet::help()
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > -------------------------+\n");
    printf("| [0]  Stand-by                 | [1]  Grab Bullets                |\n");
	printf("| [2]  Request Control          | [3]  Takeoff                     |\n");
    printf("| [4]  To Normal Altitude       | [5]  Fly to Car                  |\n");
    printf("| [6]  Find Car                 | [7]  Serve Car                   |\n");
	printf("| [8]  Drop bullets             | [9]  Back to Normal Altitude     |\n");	
	printf("| [a]  Fly Back                 | [b]  Find Park                   |\n");	
    printf("| [c]  Visual Servo Landing     | [d]  Landing                     |\n");	
    printf("| [e]  Release Control          | [f]  Jetbang Free Style          |\n");	
    printf("| [g]  Pause Free Style         | [h]  Resume Free Style           |\n");
    printf("| [i]  Cutoff Free Style        | [j]  Help                        |\n");	
    printf("+------------------------------------------------------------------+\n");
}

void Jet::spin()
{
    help();

    ros::Rate rate(spin_rate);

    while (ros::ok())
    {
        ros::spinOnce();

        char c = kbhit();

        if (c == 'f')
        {
            std::cout << "\nAction: " << "Jetbang Free Style" << std::endl;
            freestyle = true;
            jet_state = STAND_BY;
        }
        if (c == 'g')
        {
            std::cout << "\nAction: " << "Pause Free Style" << std::endl;
            freestyle = false;
        }
        if (c == 'h')
        {
            std::cout << "\nAction: " << "Resume Free Style" << std::endl;
            freestyle = true;
        }
        if (c == 'i')
        {
            std::cout << "\nAction: " << "Cutoff Free Style" << std::endl;
            freestyle = false;
            jet_state = STAND_BY;
        }

        if (freestyle)
        {
            std::cout << "Jetbang Free Styling" << std::endl;
            stateMachine();
        }

        if (c >= '0' && c <= '9')
        {
            action(c - '0');
        }
        if (c >= 'a' && c <= 'e')
        {
            action(c - 'a' + 10);
        }
        if (c == 'j')
        {
            help();
        }

        pub_jet_state();

        rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jet");

    ros::NodeHandle nh;

    Jet jet(nh);

    jet.spin();

    return 0;

}
