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
#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include "intersectionLocation.h"
#include "topoMap.h"

// using namespace perception;

std::mutex mtx_visual;
std::mutex mtx_pose;
std::vector<float> pose(3, 0.0);
std::string path = "/home/lsj/dev/Mine_WS/simu_data/";
int curVertex_ID = -1;
int preVertex_ID = -1;
bool intersectionVerified = false;
TopoMap topoMap;
pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud;
pcl::visualization::PCLVisualizer viewer("ndt");
ros::Publisher pubOdom;

void vertexIDHandler(const std_msgs::Int32 msg)
{
    curVertex_ID = msg.data;
    return;
}

void intersectionHandler(const std_msgs::Bool msg)
{
    intersectionVerified = msg.data;
    return;
}

void cloudHandler(const sensor_msgs::PointCloud2ConstPtr msg)
{
    pcl::fromROSMsg(*msg, *lidarCloud);
    ROS_DEBUG_STREAM("lidarCloud :  " << lidarCloud->size());
    return;
}

void location()
{
    float yaw_pre = topoMap.get_angleDiff(preVertex_ID, curVertex_ID);
    //加载目标点云pcd
    std::string fileName = path + std::to_string(curVertex_ID) + "_node.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
    target_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file target_cloud.pcd \n");
        return;
    }
    mtx_pose.lock();
    pose = {-10, -1, static_cast<float>(-yaw_pre * M_PI / 180)};
    mtx_pose.unlock();
    while (ros::ok())
    {
        if (!intersectionVerified)
        {
            ROS_INFO_STREAM("Leave the vertex  " << curVertex_ID);
            break;
        }
        intersectionLocation(pose, target_cloud, lidarCloud, viewer);
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
    while (ros::ok())
    {
        if (intersectionVerified)
        {
            std::thread thread2(location);
            std::thread visual_thread(visual);
            thread2.join();
            visual_thread.join();
            viewer.removeAllPointClouds();
            preVertex_ID = curVertex_ID;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "intersectionLocation");
    ros::NodeHandle nh("~");
    pubOdom = nh.advertise<nav_msgs::Odometry>("intersectionOdom", 5);
    lidarCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    ros::Subscriber subVertex = nh.subscribe<std_msgs::Int32>("vertex_ID", 1, &vertexIDHandler);
    ros::Subscriber subIntersection = nh.subscribe<std_msgs::Bool>("/perception/intersectionVerified", 1, &intersectionHandler);
    ros::Subscriber subLidarCloudCombined = nh.subscribe<sensor_msgs::PointCloud2>("cloud_Combined", 1, &cloudHandler);

    topoMap = loadMap(path + "Vertex.yaml", path + "Edge.yaml", path);

    std::thread thread1(beginLocation);

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        rate.sleep();
    }

    // /********读取数据********/
    // std::string fin = "/home/lsj/dev/Mine_WS/src/perception/include/ndtData.yaml";
    // YAML::Node config = YAML::LoadFile(fin);
    // float yaw = config["yaw"].as<float>() * M_PI / 180;
    // float x = config["x"].as<float>();
    // float y = config["y"].as<float>();
    // std::string test1File = config["test1File"].as<std::string>();
    // std::string test2File = config["test2File"].as<std::string>();
    // float yaw_pre = config["yaw_pre"].as<float>() * M_PI / 180;
    // float x_pre = config["x_pre"].as<float>();
    // float y_pre = config["y_pre"].as<float>();
    // /********读取数据********/

    return 0;
}
