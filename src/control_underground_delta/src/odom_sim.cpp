#define RATE 10

#include <ros/ros.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32MultiArray.h>

using namespace std;

struct VehState
{
    float x, y, yaw, u, w; // 横坐标, 纵坐标, 航向角, 线速度, 角速度
    // float a; // 加速度
};

VehState state_now;

float Veh_L = 2.14;

float cmd_u, cmd_w;

void CmdCallback(const std_msgs::Float32MultiArray& msg)
{
    cmd_u = msg.data[0];
    cmd_w = state_now.u * tan(msg.data[1])/Veh_L;
    // cmd_w = msg.data[1];
}

void OdomUpdate(void)
{
    state_now.x = state_now.x + state_now.u*cos(state_now.yaw)/RATE;
    
    state_now.y = state_now.y + state_now.u*sin(state_now.yaw)/RATE;
    
    state_now.yaw = state_now.yaw + state_now.w/RATE;

    state_now.u = cmd_u;

    state_now.w = cmd_w;
    
    while (state_now.yaw > M_PI)
        state_now.yaw = state_now.yaw - 2*M_PI;
    
    while (state_now.yaw <= -M_PI)
        state_now.yaw = state_now.yaw + 2*M_PI;
}

nav_msgs::Odometry odom;

void OdomMsgGen(void)
{
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = state_now.x;
    odom.pose.pose.position.y = state_now.y;
    odom.twist.twist.linear.x = state_now.u;
    odom.twist.twist.angular.z = state_now.w;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(state_now.yaw);
    
    ROS_INFO("--------------------------------------");
    ROS_INFO("x = %f, y = %f", state_now.x, state_now.y);
    ROS_INFO("u = %f, w = %f, yaw = %f", state_now.u, state_now.w, state_now.yaw);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odom_sim");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/cmd_delta", 10, CmdCallback);
    // ros::Subscriber sub = nh.subscribe("/cmd", 10, CmdCallback);    
    ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    
    odom.header.frame_id = "map";

    state_now.x = 0;
    state_now.y = 0;
    state_now.yaw = 0;
    state_now.u = 0;
    state_now.w = 0;

    ros::Rate loop_rate(RATE);
    while (ros::ok())
    {
        ros::spinOnce();

        OdomUpdate();

        OdomMsgGen();

        pub.publish(odom);

        loop_rate.sleep();
    }
    
    return 0;
}