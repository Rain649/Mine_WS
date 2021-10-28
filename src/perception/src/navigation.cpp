#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>           // 包含kdtree头文件
#include <pcl/kdtree/kdtree_flann.h>     //kdtree搜索
#include <pcl/filters/extract_indices.h> //按索引提取
#include <pcl/filters/passthrough.h>     //直通滤波

#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include "intersectionLocation.h"
#include "topoMap.h"

// using namespace perception;

std::mutex mtx_visual;
std::vector<float> pose(3, 0.0);
// std::vector<int> path;
std::vector<int> path{2, 11, 12, 13, 16, 17, 18, 1};
std::string dataPath = "simu_data/";
int preVertex_index = 0;
bool intersectionVerified = false;
TopoMap topoMap;
pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud;
pcl::visualization::PCLVisualizer viewer("ndt");
ros::Publisher pubOdom;
ros::Publisher pubIntersectionID;

//将弧度转换到-π~π区间
inline void radianTransform(float &radian)
{
    while (radian > M_PI)
        radian -= 2 * M_PI;
    while (radian <= -M_PI)
        radian += 2 * M_PI;
}

void pathHandler(const std_msgs::Int32MultiArray msg)
{
    if (!path.empty())
        return;
    for (int i = 0; i < msg.data.size(); ++i)
    {
        path.push_back(msg.data[i]);
    }
    return;
}

void intersectionHandler(const std_msgs::Bool msg)
{
    intersectionVerified = msg.data;
    return;
}

void cloudHandler(const sensor_msgs::PointCloud2ConstPtr msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCombined(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudNoCar(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg, *cloudCombined);
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_1;
    std::vector<int> index_1;      //保存每个近邻点的索引
    std::vector<float> distance_1; //保存每个近邻点与查找点之间的欧式距离平方
    // 外围分割
    kdtree_1.setInputCloud(cloudCombined); // 设置要搜索的点云，建立KDTree
    pcl::ExtractIndices<pcl::PointXYZ> extract_1;
    pcl::PointXYZ zeroPoint;
    zeroPoint.x = zeroPoint.y = zeroPoint.z = 0;
    int segmentationRadius = 20;
    if (kdtree_1.radiusSearch(zeroPoint, segmentationRadius, index_1, distance_1) == 0)
    {
        ROS_ERROR("There is no point nearby !!!");
        return;
    }
    extract_1.setInputCloud(cloudCombined);
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index_1);
    extract_1.setIndices(index_ptr);
    extract_1.setNegative(false); //如果设为true,可以提取指定index之外的点云
    extract_1.filter(*cloudNoCar);

    pcl::PassThrough<pcl::PointXYZ> groundFilter;
    groundFilter.setInputCloud(cloudNoCar);
    groundFilter.setFilterFieldName("z");
    groundFilter.setFilterLimits(-0.8, 10);
    groundFilter.setFilterLimitsNegative(false);
    groundFilter.filter(*lidarCloud);

    ROS_DEBUG_STREAM("lidarCloud :  " << lidarCloud->size());
    return;
}

void location()
{
    float yaw_pre;
    std::string fileName;
    if (path[preVertex_index] == 0)
    {
        fileName = dataPath + std::to_string(path[preVertex_index] + 1) + "_node.pcd";
        yaw_pre = 0;
    }
    else
    {
        yaw_pre = topoMap.get_angleDiff(path[preVertex_index], path[preVertex_index + 1]);
        fileName = dataPath + std::to_string(path[preVertex_index + 1]) + "_node.pcd";
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
    target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file target_cloud.pcd \n");
        return;
    }
    pose[0] = -10 * cos(yaw_pre);
    pose[1] = -10 * sin(yaw_pre);
    pose[2] = yaw_pre;
    radianTransform(pose[2]);
    while (ros::ok())
    {
        if (!intersectionVerified)
        {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            if (!intersectionVerified)
            {
                ROS_ERROR_STREAM("Leave the vertex  " << path[preVertex_index + 1]);
                viewer.removeAllPointClouds();
                break;
            }
        }
        mtx_visual.lock();
        intersectionLocation(pose, target_cloud, lidarCloud, viewer);
        mtx_visual.unlock();
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose[2]);
        odom.pose.pose.orientation.x = geoQuat.x;
        odom.pose.pose.orientation.y = geoQuat.y;
        odom.pose.pose.orientation.z = geoQuat.z;
        odom.pose.pose.orientation.w = geoQuat.w;
        odom.pose.pose.position.x = pose[0];
        odom.pose.pose.position.y = pose[1];
        odom.pose.pose.position.z = 0;
        pubOdom.publish(odom);
        // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return;
}

void visual()
{
    while (!viewer.wasStopped())
    {
        mtx_visual.lock();
        viewer.spinOnce(10);
        mtx_visual.unlock();
    }
    return;
}

void beginLocation()
{
    std::thread visual_thread(visual);
    while (ros::ok())
    {
        if (intersectionVerified)
        {
            if (preVertex_index == path.size() - 2)
            {
                ROS_INFO("Arrive Destination !!!");
                break;
            }
            std::thread thread2(location);
            thread2.join();
            preVertex_index += 1;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    visual_thread.join();
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation");
    ros::NodeHandle nh("~");
    pubOdom = nh.advertise<nav_msgs::Odometry>("intersectionOdom", 1);
    pubIntersectionID = nh.advertise<std_msgs::Int32>("intersection_id", 1);
    lidarCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // 初始化点云可视化界面
    int v1(1); //设置左窗口
    int v2(2); //设置右窗口
    viewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
    viewer.setBackgroundColor(0, 0, 0, v1);
    viewer.createViewPort(0.5, 0.0, 1, 1, v2);
    viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);
    viewer.addCoordinateSystem(5);
    viewer.initCameraParameters();
    viewer.setCameraPosition(30, 40, 50, -3, -4, -5, 0);

    // ros::Subscriber subVertex = nh.subscribe<std_msgs::Int32>("vertex_ID", 1, &vertexIDHandler);
    ros::Subscriber subIntersection = nh.subscribe<std_msgs::Bool>("/intersection/intersectionVerified", 1, &intersectionHandler);
    ros::Subscriber subLidarCloudCombined = nh.subscribe<sensor_msgs::PointCloud2>("/simuSegSave/cloud_Combined", 1, &cloudHandler);
    ros::Subscriber subPath = nh.subscribe<std_msgs::Int32MultiArray>("/pathArray", 1, &pathHandler);

    topoMap = loadMap(dataPath + "Vertex.yaml", dataPath + "Edge.yaml", dataPath);

    std::thread thread1(beginLocation);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        std_msgs::Int32 intersectionID;
        if (!path.empty())
            intersectionID.data = path[preVertex_index + 1];
        else if (preVertex_index + 1 >= path.size())
        {
            intersectionID.data = path.back();
        }
        else
        {
            intersectionID.data = -1;
            ROS_ERROR_THROTTLE(2, "Intersection ID ERROR!!!");
        }
        pubIntersectionID.publish(intersectionID);

        rate.sleep();
    }

    return 0;
}
