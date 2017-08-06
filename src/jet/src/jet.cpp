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

    ros::NodeHandle np("~");

    np.param<int>("spin_rate", spin_rate, 50);
    np.param<std::string>("serial_port", serial_port, "/dev/ttyTHS2"); 
    np.param<int>("serial_baudrate", serial_baudrate, 115200);

    int ret = uart_open(&uart_fd, serial_port.c_str(), serial_baudrate, UART_OFLAG_WR);
    if (ret < 0) {
        fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , serial_port.c_str());
    }

    cmd_grabber_srv = nh.advertiseService("/grabber/cmd", &Jet::cmd_grabber_callback, this);
    stat_grabber_srv = nh.advertiseService("/grabber/stat", &Jet::stat_grabber_callback, this);

    reload_pid_param_srv = nh.advertiseService("/reload_pid_param", &Jet::reload_pid_param_callback, this);
    reload_vision_param_srv = nh.advertiseService("/reload_vision_param", &Jet::reload_vision_param_callback, this);
    reload_flight_param_srv = nh.advertiseService("/reload_flight_param", &Jet::reload_flight_param_callback, this);
    reload_dropoint_param_srv = nh.advertiseService("/reload_dropoint_param", &Jet::reload_dropoint_param_callback, this);
    

    odometry_sub = nh.subscribe("/guidance/odom", 10, &Jet::odometry_callback, this);
    vision_sub = nh.subscribe("/vision/pos", 10, &Jet::vision_callback, this);

    jet_state_pub = nh.advertise<std_msgs::UInt8>("/jet_state", 10);
    odom_calied_pub = nh.advertise<nav_msgs::Odometry>("/odom_calied", 10);

    load_pid_param(nh);
    load_vision_param(nh);
    load_flight_param(nh);
    load_dropoint_param(nh);

    jet_nav_action_server = new JetNavActionServer(nh,
            "jet/nav_action",
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
    std::cout << "takeoff_height: " << takeoff_height << std::endl;
    std::cout << "landing_height: " << landing_height << std::endl;
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
    pub_odom_calied();
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

void Jet::vision_callback(const geometry_msgs::Vector3Stamped& position)
{
    vision_target_pos = position;
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

bool Jet::cmd_grabber(uint8_t c)
{
    uint8_t buf[4];
    buf[0] = 0xa5;
    buf[1] = c;
    buf[2] = 0xfe;
    CRC8Append(buf, 4, 0xff);
    return (write(uart_fd, buf, 4) == 4);
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

    std::cout << "------loop control: " << std::endl;
    std::cout << "rx: " << rx << ", ry: " << ry << ", rz: " << rz << ", ryaw: " << ryaw << std::endl;
    std::cout << "fx: " << fx << ", fy: " << fy << ", fz: " << fz << ", fyaw: " << fyaw << std::endl;
    std::cout << "ex: " << x << ", ey: " << y << ", ez: " << z << ", eyaw: " << yaw << std::endl;
    std::cout << "ox: " << ox << ", oy: " << oy << ", oz: " << oz << ", oyaw: " << oyaw << std::endl;

    return goal_reached();
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
        std::cout << "\nAction: " << "Grab Bullets" << std::endl;
        return doGrabBullets();

        case REQUEST_CONTROL:
        std::cout << "\nAction: " << "Request Control" << std::endl;
        return doRequestControl();

        case TAKE_OFF:
        std::cout << "\nAction: " << "Takeoff" << std::endl;
        return doTakeoff();

        case FLY_TO_CAR:
        std::cout << "\nAction: " << "Fly to Car" << std::endl;
        return doFlyToCar();

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

        case VISUAL_SERVO_LANDING:
        std::cout << "\nAction: " << "Visual Servo Landing" << std::endl;
        return doVisualServoLanding();

        case LANDING:
        std::cout << "\nAction: " << "Landing" << std::endl;
        return doLanding();

        default:
        return false;
    }
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

        float x = dst_x - odom_calied.pose.pose.position.x;
        float y = dst_y - odom_calied.pose.pose.position.y;
        float z = dst_z - odom_calied.pose.pose.position.z;
        float yaw = dst_yaw - tf::getYaw(odom_calied.pose.pose.orientation);

        pid_control(1, x, y, z, yaw);

        float det_x = (100 * x) / dis_x;
        float det_y = (100 * y) / dis_y;
        float det_z = (100 * z) / dis_z;
        float det_yaw = (100 * yaw) / dis_yaw;

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

void Jet::stateMachine()
{
}

void Jet::spin()
{
    help();

    ros::Rate rate(spin_rate);

    while (ros::ok())
    {
        ros::spinOnce();

        char c = kbhit();

        if (c >= '1' && c <= '9')
        {
            action(c - '1');
        }
        if (c >= 'a' && c <= 'a')
        {
            action(c - 'a' + 9);
        }
        if (c == 'h')
        {
            help();
        }
        
        jet_state = status();
        pub_jet_state();

        rate.sleep();
    }
}

void Jet::help()
{
    printf("\r\n");
    printf("+-------------------------- < Main menu > -------------------------+\n");
	printf("| [1]  Grab Bullets             | [2]  Request Control             |\n");
	printf("| [3]  Takeoff                  | [4]  Fly to Car                  |\n");	
	printf("| [5]  Serve Car                | [6]  Drop bullets                |\n");	
    printf("| [7]  Back to Normal Altitude  | [8]  Fly Back                    |\n");	
    printf("| [9]  Visual Servo Landing     | [a]  Land                        |\n");	
    printf("+------------------------------------------------------------------+\n");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "jet");

    ros::NodeHandle nh;

    Jet jet(nh);

    jet.spin();

    return 0;

}
