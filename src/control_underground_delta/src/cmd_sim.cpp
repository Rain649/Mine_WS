#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "nav_msgs/Odometry.h"
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>

double veh_u = 0;

std_msgs::Float32MultiArray cmd;

void OdomCallback(const nav_msgs::Odometry& msg)
{
    veh_u = msg.twist.twist.linear.x;
}

// void AEBStateCallback(const std_msgs::UInt8& msg)
// {
//     if( msg.data == 1 )
//     {
//         cmd.data[3] = 1.f;
//         cmd.data[0] = 0;
//     }
//     else
//         cmd.data[3] = 0.f;
// }

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "cmd_sim_delta");
    ros::NodeHandle nh;
    
    ros::Subscriber OdomSub = nh.subscribe("/chassis_speed_delta", 10, OdomCallback);
    // ros::Subscriber AEBSub = nh.subscribe("/aeb_state", 10, AEBStateCallback);

    ros::Publisher CmdPub = nh.advertise<std_msgs::Float32MultiArray>("/cmd_delta", 10);
    // ros::Publisher CmdPub = nh.advertise<std_msgs::Float32MultiArray>("/target_u", 10);    
    
    double dt = 0.05;

    cmd.data.resize(4);
    cmd.data[0] = 0.f; // 纵向
    cmd.data[1] = 0.f; // 横向
    cmd.data[2] = 1.f; // >0.5自动模式，<0.5手动模式
    cmd.data[3] = 0.f; // >0.5需要紧急制动，<0.5不需要紧急制动
    
    int counter = 0;
    
    bool acc = false;

    float t = 0;

    float target_u = 7/3.6;

    // gazebo
    ros::Publisher CmdSimPub;
    geometry_msgs::Twist cmd_sim;

    CmdSimPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    // gazebo

    cmd_sim.linear.x = 0;
    cmd_sim.angular.z = 0.5;

    ros::Rate loop_rate(1.f / dt);
    while (ros::ok())
    {
        CmdSimPub.publish(cmd_sim);
    }
    
    // while(ros::ok())
    // {
    //     ros::spinOnce();
        
    //     // if (fabs(veh_u - target_u) < 1/3.6)
    //     //         acc = true;

    //     // if (!acc)
    //     // {
    //     //     cmd.data[0] += 0.5/3.6;
        
    //     //     if (cmd.data[0] > target_u)
    //     //     {
    //     //         cmd.data[0] = target_u;
    //     //     }
    //     // }
    //     // else
    //     // {
    //     //     cmd.data[0] = 3/3.6 * sin( 2*M_PI / 10 * t ) + target_u;
    //     //     t += dt;
    //     // }
                
    //     // cmd.data[0] = 5/3.6;

    //     // 前轮摆角
    //     // cmd.data[1] = 20*M_PI/180;

    //     counter++;

    //     // cmd.data[0] = 30;

    //     if (counter < 100)
    //     {
    //         cmd.data[0] = 40;
    //     }
    //     else
    //     {
    //         counter = 100;

    //         cmd.data[0] = 25; //17
    //     }

    //     // if (counter < 500)
    //     // {
    //     //     cmd.data[0] = 30;//25;
    //     // }
    //     // else
    //     // {
    //     //     counter = 500;

    //     //     // cmd.data[0] = -60;
    //     //     cmd.data[0] = 0;            
    //     // }

    //     // cmd.data[1] = 15*M_PI/180;

    //     CmdPub.publish(cmd);

    //     // printf("cmd_u = %lf\n", cmd.data[0]);

    //     printf("cmd_torque = %lf\n", cmd.data[0]);

    //     printf("cmd_angle = %lf\n", cmd.data[1]);

    //     // printf("veh_u = %lf\n", veh_u);
    //     std::cout << "veh_u = " << veh_u << "m/s, " << veh_u*3.6 << "km/h" << std::endl;
        
    //     // printf("error = %lf\n", cmd.data[0] - veh_u);
        
    //     // std::cout << "aeb = " << cmd.data[3] << std::endl;
        
    //     loop_rate.sleep();

    //     std::cout << std::endl;
    // }

    cmd.data[0] = 0;
    cmd.data[1] = 0;
    cmd.data[2] = 0;
    cmd.data[3] = 0;

    CmdPub.publish( cmd );
    
    return 0;
}