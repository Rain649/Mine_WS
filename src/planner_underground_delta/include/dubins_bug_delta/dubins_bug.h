#ifndef __DB_PLANNER__
#define __DB_PLANNER__

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vector>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <fstream>

#include "r_s_planner/dubins.h"

typedef pcl::PointXYZI PointType;

// // Dubins曲线
// class DubinsStateSpace;

class VehState
{
    public:
        // 横坐标, 纵坐标, 航向角, 线速度, 角速度
        float x, y, yaw, u, w;
};

class PathNode
{
    public:
        // 横坐标, 纵坐标, 距起点长度, 切向角, 曲率
        float x, y, s, phi, c;
};

class LaserPoint
{
    public:
        // 在车辆坐标系下的坐标，航向
        float x_veh, y_veh, yaw_veh;

        // 距车辆的距离，距目标的距离
        float dis_veh, dis_tar;
};

class LaserPoint_dis: public LaserPoint
{
    public:
        float dis_cost;

    bool operator<(const LaserPoint_dis& other) const
    {
        return dis_cost > other.dis_cost; // 小顶堆
    }
};

class DBPlanner
{
    public:
        DBPlanner(ros::NodeHandle& nh);

        bool CallbackFlag(void);

        // publish
        void PubPathMsg(void);

        // based on TangentBug
        void BugPlanner(DubinsStateSpace& db);

        void MotionToGoal(DubinsStateSpace& db);

        // 在车辆坐标系下进行碰撞检测，返回碰撞点下标
        int ObsCheck(std::vector<PathNode> &path, int near_index);

        // 坐标变换
        void PathWorld2Veh(std::vector<PathNode> &path_w, std::vector<PathNode> &path_v);

        void PathVeh2World(std::vector<PathNode> &path_w, std::vector<PathNode> &path_v);

        // 更新预测路径
        bool UpdatePdtPath(void);

        void GenPdtPath(void);

        // 返回最近路点下标
        int FindNearPoint(std::vector<PathNode> &path, VehState &veh);

        // 计算路点参数
        void GenPathPara(std::vector<PathNode> &path);

        // Dubins/Reeds-Shepp曲线
        double start_state[3], start_state_veh[3];
        double final_state[3], final_state_veh[3];
        double steer_radius;
        double step_size;
        double db_rs_length;
        std::vector<std::vector<double> > db_rs_path;
        bool db_rs_flag;

        // callback flag
        bool odom_flag, target_flag, lidar_flag;

        nav_msgs::Path path_pdt_veh_msg; // path

        int odom_counter, target_counter, lidar_counter;

        int Counter_MAX;

        void ResetDB(void);

    private:
        // subscribe
        ros::Subscriber OdomSub, TargetSub, LidarSub;
        
        void OdomCallback(const nav_msgs::Odometry& msg);
        
        void TargetCallback(const nav_msgs::Odometry& msg);

        void LidarCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

        // publish
        ros::Publisher MapPub, LaserPointPub, LaserEdgePointPub;

        ros::Publisher PathLocalPub, PathLocalVehPub, PathPdtPub, PathPdtVehPub;

        ros::Publisher O2tMinPub, O2tMinVehPub, LeaveEdgePub, LeaveVehPub;

        // test
        ros::Publisher TargetInVehPub;
        nav_msgs::Odometry target_veh_msg;
        // test

        // msg
        nav_msgs::OccupancyGrid map_msg; // map
        
        geometry_msgs::PoseArray laser_point_msg, laser_edge_msg; // lidar
        
        nav_msgs::Path path_local_msg, path_local_veh_msg; // path

        nav_msgs::Path path_pdt_msg; // path
                
        // vehicle
        VehState current_state;

        float Delta_f_max;  // 前轮最大摆角

        float Veh_L;        // 车辆轴距

        double Lf, Lr;

        // lidar
        pcl::PointCloud<PointType>::Ptr LaserCloudSurround;
        pcl::PointCloud<PointType>::Ptr LaserCloudSurroundFiltered;
        pcl::PassThrough<PointType> pass_x;
        pcl::PassThrough<PointType> pass_y;
        pcl::PassThrough<PointType> pass_z;
        pcl::VoxelGrid<PointType> downSizeFilter;
        
        PointType pointSub;

        float Laser_Edge;   // 雷达范围

        void GetLaserEdgePoint(void);

        float ANGLE_RES;    // 角度分辨率，弧度

        int seg_num;        // 分区个数
        
        float horizon_angle;

        int horizon_index;

        std::vector<LaserPoint> laser_point; // 各方向上最近的雷达点

        std::vector<LaserPoint> laser_edge_point; // 雷达范围边界点，障碍物边界旋转点

        LaserPoint laser_point_tmp;

        int Neighbor_num;       // 边界检测相邻点的个数，偶数

        float Edge_tolerance;   // 边界判断阈值
        
        bool obs_edge_cw_flag, obs_edge_ccw_flag;

        float theta_safe, k_safe;

        float k_sample;
        
        std::vector<float> neighbor_dis;
        
        std::vector<bool> neighbor_flag;

        std::priority_queue<LaserPoint_dis> free_point_dis; // 无障碍边界点，总距离小顶堆

        LaserPoint_dis free_dis_tmp;

        // map
        std::vector<signed char> map_vec;
        
        cv::Mat image_map;  // 二值化地图

        cv::Mat image_dis;  // 距离地图
        
        int map_rows, map_cols; // 地图行数、列数

        float map_o_x, map_o_y; // 原点

        float map_res;          // 分辨率
        
        int index_row, index_col;

        // path
        std::vector<PathNode> path_local, path_local_veh;
        
        std::vector<PathNode> path_tmp, path_tmp_veh;

        std::vector<PathNode> path_pdt, path_pdt_veh;

        PathNode node_tmp;

        // dis vars
        float dis_min, dis_tmp;
        
        float Infinity;

        // dis tolerance
        float Tar_tolerance;    // 到达目标阈值

        // based on TangentBug
        float dis_v2t_now, dis_v2t_old;     // 车辆到目标点的距离

        float target_x_veh, target_y_veh;   // 在车辆坐标系下目标点的坐标

        // 碰撞检测
        int obs_index;      // 碰撞点下标

        float obs_path_len; // 碰撞检测路径长度

        float OBS_DIS;      // 碰撞检测距离

        // 更新预测路径
        float Phi_alter;    // 规划航向变化阈值

        int near_index;     // 最近点下标

        float s_threshold;  // 更新路径距离阈值

        int S_proportion;
};

#endif //__DB_PLANNER__