#ifndef __PATH_FOLLOW__
#define __PATH_FOLLOW__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>

#include <vector>
#include <deque>
#include <fstream>

class VehState
{
    public:
        float x, y, yaw, u, w;
};

class PathNode
{
    public:
        float x, y, s, phi, c;

        float u;
};

class PathFollow
{
    public:
        PathFollow(ros::NodeHandle& nh);
        
        bool CallbackFlag(void);

        void PushPathNode(void);

        void GenLatCmd(void);

        void GenLonCmd(void);

        void PubCmd(void);

        void Stop(void);

        float RATE;

    private:
        // subscribe
        ros::Subscriber OdomSub;
        void OdomCallback(const nav_msgs::Odometry& msg);

        ros::Subscriber PathSub;
        void PathCallback(const nav_msgs::Path& msg);

        int path_counter;
        int Counter_MAX;

        ros::Subscriber VehSub;
        void VehCallback(const nav_msgs::Odometry& msg);

        ros::Subscriber PIDSub;
        void PIDCallback(const geometry_msgs::TwistStamped& msg);

        ros::Subscriber TargetUSub;
        void TargetUCallback(const std_msgs::Float32MultiArray& msg);
        
        // flag
        bool odom_flag;
        bool path_flag;
        bool veh_flag;

        bool pid_flag;
        bool u_aim_flag;

        // publish
        ros::Publisher CmdPub;
        std_msgs::Float32MultiArray cmd;

        ros::Publisher CmdSimPub;
        geometry_msgs::Twist cmd_sim;
        float cmd_sim_lon;

        // path
        std::vector<PathNode> path_ref;
        
        PathNode node_tmp;

        // vehicle
        VehState state_now;

        float Veh_L;

        float Steer_max;        

        // 根据路径曲率计算参考车速
        float phi_delta;

        float U_max, U_min;

        float C_max;

        // 横向，预瞄
        int near_num;       // 最近点下标

        int aim_num;        // 预瞄点下标

        float pre_dis;      // 预瞄距离

        float pre_dis_min;

        float u_weight;     // 车速系数

        float aim_dis;      // 与预瞄点的距离

        float pre_angle;    // 与预瞄点的夹角

        float pre_r;        // 预瞄半径

        float dis_min, dis_tmp;

        float cmd_steer;

        // 纵向，PID
        int counter;

        float u_sum;

        float u_aim;

        float error_u, error_u_old;

        float error_sum;

        std::deque<float> error_deque;

        int Error_size_max;

        float pid_p, pid_i, pid_d;

        float cmd_torque, cmd_torque_old;

        float Torque_max, Torque_min;

        float Torque_delta;

        float Torque_startup;

        float Near_dis;
};

#endif // __PATH_FOLLOW__