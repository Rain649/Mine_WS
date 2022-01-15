#include "control_underground_delta/path_follow.h"

PathFollow::PathFollow(ros::NodeHandle& nh)
{
    // loop rate
    RATE = 20;

    // subscribe
    // OdomSub = nh.subscribe("/odom", 10, &PathFollow::OdomCallback, this);

    std::string topic_path_ug_vehicle;
    nh.getParam("topic_path_ug_vehicle", topic_path_ug_vehicle);
    PathSub = nh.subscribe(topic_path_ug_vehicle, 10, &PathFollow::PathCallback, this);
    // PathSub = nh.subscribe("/path_ug_vehicle", 10, &PathFollow::PathCallback, this);

    path_counter = 0;
    Counter_MAX = 1 * RATE;

    std::string topic_chassis;
    nh.getParam("topic_chassis", topic_chassis);
    VehSub = nh.subscribe(topic_chassis, 10, &PathFollow::VehCallback, this);
    // VehSub = nh.subscribe("/chassis_speed_delta", 10, &PathFollow::VehCallback, this);
    
    // PIDSub = nh.subscribe("/pid_teleop", 10, &PathFollow::PIDCallback, this);
    
    // TargetSub = nh.subscribe("/u_aim", 10, &PathFollow::TargetUCallback, this);
    
    // flag
    // odom_flag = false;
    path_flag = false;
    veh_flag = false;

    pid_flag = false;
    u_aim_flag = false;

    // publish
    std::string topic_cmd;
    nh.getParam("topic_cmd", topic_cmd);
    CmdPub = nh.advertise<std_msgs::Float32MultiArray>(topic_cmd, 10);
    // CmdPub = nh.advertise<std_msgs::Float32MultiArray>("/cmd_delta", 10);

    cmd.data.resize(4);
    cmd.data[0] = 0;    // 电机力矩
    cmd.data[1] = 0;    // 前轮转角
    cmd.data[2] = 1;    // > 0.5 自动模式，< 0.5 手动模式
    cmd.data[3] = 0;    // > 0.5 需要紧急制动，< 0.5 不需要紧急制动

    // vehicle
    nh.getParam("Veh_L", Veh_L);
    // Veh_L = 3.5;

    Steer_max = 30*M_PI/180;

    // sim cmd
    CmdSimPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
    cmd_sim.linear.x = 0;
    cmd_sim.angular.z = 0;

    nh.getParam("cmd_sim_lon", cmd_sim_lon);
    cmd_sim_lat = cmd_sim_lon * tan(Steer_max) / Veh_L;

    // 根据路径曲率计算参考车速
    nh.getParam("U_max", U_max);
    U_max /= 3.6;

    nh.getParam("U_min", U_min);
    U_min /= 3.6;
    
    C_max = tan(Steer_max)/Veh_L;

    // 预瞄
    nh.getParam("pre_dis_min", pre_dis_min);
    // pre_dis_min = 5;

    u_weight = 0.7;

    // PID
    counter = 0;

    u_aim = 0;

    error_u_old = 0;
    
    pid_p = 60;
    pid_i = 0.1;
    pid_d = 0.05;

    Error_size_max = 3*RATE;

    nh.getParam("Torque_max", Torque_max);
    // Torque_max = 60;

    nh.getParam("Torque_min", Torque_min);
    // Torque_min = -20;

    Torque_delta = 2;

    Torque_startup = 40;

    cmd_torque_old = 0;

    Near_dis = 5;

    state_now.x = 0;
    state_now.y = 0;
    state_now.yaw = 0;
}

// void PathFollow::OdomCallback(const nav_msgs::Odometry& msg)
// {
//     state_now.x = msg.pose.pose.position.x;

//     state_now.y = msg.pose.pose.position.y;

//     state_now.yaw = tf::getYaw(msg.pose.pose.orientation);

//     // state_now.u = msg.twist.twist.linear.x;    

//     odom_flag = true;
// }

void PathFollow::VehCallback(const nav_msgs::Odometry& msg)
{
    state_now.u = msg.twist.twist.linear.x;

    veh_flag = true;
}

void PathFollow::PathCallback(const nav_msgs::Path& msg)
{
    if (msg.poses.size() < 3)
    {
        path_flag = false;

        return;
    }
    
    path_ref.resize(msg.poses.size());

    for (size_t i = 0; i < path_ref.size(); i++)
    {
        path_ref[i].x = msg.poses[i].pose.position.x;

        path_ref[i].y = msg.poses[i].pose.position.y;
    }
    
    for (size_t i = 0; i < path_ref.size(); i++)
    {
        if (i == 0)
        {
            path_ref[i].s = 0;

            path_ref[i].phi = atan2(path_ref[i+1].y - path_ref[i].y, path_ref[i+1].x - path_ref[i].x);
        }
        else
        {
            path_ref[i].s = path_ref[i-1].s + sqrt(pow((path_ref[i].y - path_ref[i-1].y), 2) + pow((path_ref[i].x - path_ref[i-1].x), 2));

            if (i == path_ref.size() -1)
            {
                path_ref[i].phi = atan2(path_ref[i].y - path_ref[i-1].y, path_ref[i].x - path_ref[i-1].x);
            }
            else
            {
                path_ref[i].phi = atan2(path_ref[i+1].y - path_ref[i-1].y, path_ref[i+1].x - path_ref[i-1].x);
            }
            
            phi_delta = path_ref[i].phi - path_ref[i-1].phi;

            if (phi_delta > M_PI)
            {
                phi_delta = phi_delta - 2*M_PI;
            }

            if (phi_delta <= -M_PI)
            {
                phi_delta = phi_delta + 2*M_PI;
            }
            
            path_ref[i].c = phi_delta/(path_ref[i].s - path_ref[i-1].s);
        }
    }
    
    path_ref[0].c = path_ref[1].c;

    // 根据路径点曲率计算参考车速
    for (size_t i = 0; i < path_ref.size(); i++)
    {
        if (fabs(path_ref[i].c) >= C_max)
        {
            path_ref[i].u = U_min;
        }
        else
        {
            path_ref[i].u = (U_min - U_max)/pow(C_max, 2) * pow(fabs(path_ref[i].c), 2) + U_max;
            //(U_min - U_max)/C_max * fabs(path_ref[i].c) + U_max;
        }
    }
    
    path_flag = true;

    // std::cout << path_ref.back().s << std::endl;

    path_counter = 0;
}

// void PathFollow::PIDCallback(const geometry_msgs::TwistStamped& msg)
// {
//     pid_p = msg.twist.linear.x;
    
//     pid_i = msg.twist.linear.y;

//     pid_d = msg.twist.linear.z;

//     pid_flag = true;

//     std::cout << "receive pid from teleop" << std::endl;
// }

// void PathFollow::TargetUCallback(const std_msgs::Float32MultiArray& msg)
// {
//     u_aim = msg.data[0];

//     u_aim_flag = true;
// }