#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
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
#include <pcl/filters/filter.h>

#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include "intersectionLocation.h"
#include "topoMap.h"
#include "registrationConfig.h"

#include <unistd.h>

inline void
radianTransform(float &radian)
{
    while (radian > M_PI)
        radian -= 2 * M_PI;
    while (radian <= -M_PI)
        radian += 2 * M_PI;
}

/**
 * Load registration configuration from yaml file
 * @param dataPath path of data file
 */
RegistrationConfig loadRegistrationConfig(std::string dataPath)
{
    std::string fin = dataPath + "registrationConfig.yaml";
    YAML::Node config = YAML::LoadFile(fin);
    RegistrationConfig ret;
    ret.x_pre = config["x"].as<float>();
    ret.y_pre = config["y"].as<float>();
    ret.yaw_pre = config["yaw"].as<float>() * M_PI / 180;
    radianTransform(ret.yaw_pre);
    ret.maximumIterations = config["maximumIterations"].as<float>();
    ret.resolution = config["resolution"].as<float>();
    ret.stepSize = config["stepSize"].as<float>();
    ret.transformationEpsilon = config["transformationEpsilon"].as<float>();
    ret.maxCorrespondenceDistance = config["maxCorrespondenceDistance"].as<float>();
    ret.euclideanFitnessEpsilon = config["euclideanFitnessEpsilon"].as<float>();
    ret.yaw_thre = config["yaw_thre"].as<double>();
    ret.fitnessScore_thre = config["fitnessScore"].as<double>();
    ret.minFitnessScore_thre = config["minFitnessScore_thre"].as<double>();
    ret.menu_bool = config["menu_bool"].as<bool>();

    return ret;
}

class Navigation
{
private:
    // 初始化点云可视化界面
    int v1{1}; //设置左窗口
    int v2{2}; //设置右窗口

    RegistrationConfig config;

    std::mutex mtx_visual;
    std::mutex mtx_cloud;
    std::vector<float> pose;
    std::vector<int> path{0, 1, 2, 11, 12, 13, 16, 17, 18, 1};
    std::string dataPath;
    int preVertex_index = 0;
    bool intersectionVerified = false;
    bool pathReceived = false;
    TopoMap topoMap;

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud;

    pcl::visualization::PCLVisualizer viewer;

    ros::NodeHandle nh;
    ros::Publisher pubOdom;
    ros::Publisher pubIntersectionID;

    ros::Subscriber subIntersection;
    ros::Subscriber subLidarCloudCombined;
    ros::Subscriber subPath;

public:
    Navigation() : nh("~")
    {
        //加载参数
        nh.param<std::string>("simu_data_path", dataPath, "src/perception/simu_data/");
        ROS_INFO("simu_data path: %s", dataPath.c_str());
        // 分配内存
        pose.resize(3);
        lidarCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

        // 设置点云配准可视化窗口
        viewer.setWindowName("registration");
        viewer.createViewPort(0.0, 0.0, 0.5, 1, v1);
        viewer.setBackgroundColor(0, 0, 0, v1);
        viewer.createViewPort(0.5, 0.0, 1, 1, v2);
        viewer.setBackgroundColor(0.5, 0.5, 0.5, v2);
        viewer.addCoordinateSystem(5);
        viewer.initCameraParameters();
        viewer.setCameraPosition(30, 40, 50, -3, -4, -5, 0);

        // 加载拓扑地图
        // std::cout << "path : " << getcwd(NULL, 0) << std::endl;
        topoMap = loadMap(dataPath + "Vertex.yaml", dataPath + "Edge.yaml", dataPath);

        // 定义topic收发
        pubOdom = nh.advertise<nav_msgs::Odometry>("intersectionOdom", 1);
        pubIntersectionID = nh.advertise<std_msgs::Int32>("intersection_id", 1);
        subIntersection = nh.subscribe<std_msgs::Bool>("/intersectionDetection/intersectionVerified", 1, &Navigation::intersectionHandler, this);
        subLidarCloudCombined = nh.subscribe<sensor_msgs::PointCloud2ConstPtr>("/lidarCloudProcess/cloud_Combined", 1, &Navigation::cloudHandler, this);
        subPath = nh.subscribe<std_msgs::Int32MultiArray>("/pathArray", 1, &Navigation::pathHandler, this);
    }

    void pathHandler(const std_msgs::Int32MultiArray msg)
    {
        if (pathReceived)
            return;
        else
        {
            path.clear();
            std::cout << "< Path";
            for (int i = 0; i < msg.data.size(); ++i)
            {
                std::cout << " > " << msg.data[i];
                path.push_back(msg.data[i]);
            }
            std::cout << std::endl;
            ;
            pathReceived = true;
        }
    }

    void intersectionHandler(const std_msgs::Bool msg)
    {
        intersectionVerified = msg.data;
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCombined(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloudCombined);

        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudTemp(new pcl::PointCloud<pcl::PointXYZ>());
        // pcl::fromROSMsg(*msg, *cloudTemp);
        // std::vector<int> mapping;
        // pcl::removeNaNFromPointCloud(*cloudTemp, *cloudCombined, mapping);

        if (cloudCombined->empty())
        {
            return;
        }
        std::vector<int> index_1;      //保存每个近邻点的索引
        std::vector<float> distance_1; //保存每个近邻点与查找点之间的欧式距离平方
        mtx_cloud.lock();
        lidarCloud->clear();
        // 外围分割
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_1;
        kdtree_1.setInputCloud(cloudCombined); // 设置要搜索的点云，建立KDTree
        pcl::ExtractIndices<pcl::PointXYZ> extract_1;
        pcl::PointXYZ zeroPoint;
        zeroPoint.x = zeroPoint.y = zeroPoint.z = 0;
        int segmentationRadius = 25;
        if (kdtree_1.radiusSearch(zeroPoint, segmentationRadius, index_1, distance_1) == 0)
        {
            ROS_ERROR("There is no point nearby !!!");
            return;
        }
        extract_1.setInputCloud(cloudCombined);
        boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index_1);
        extract_1.setIndices(index_ptr);
        extract_1.setNegative(false); //如果设为true,可以提取指定index之外的点云
        extract_1.filter(*lidarCloud);
        lidarCloud->header.stamp = cloudCombined->header.stamp;
        mtx_cloud.unlock();

        return;
    }

    /**
     * Location at intersection
     */
    void location()
    {
        // 加载目标点云，获取点云配准先验位姿
        float yaw_pre;
        std::string fileName;
        if (path[preVertex_index] == 0)
        {
            yaw_pre = 0;
        }
        else
        {
            yaw_pre = topoMap.get_angleDiff(path[preVertex_index], path[preVertex_index + 1]);
        }
        fileName = dataPath + std::to_string(path[preVertex_index + 1]) + "_node.pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        target_cloud->header.frame_id = "vehicle_base_link";
        pcl::PointCloud<pcl::PointXYZ> cloudTemp;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, cloudTemp) == -1)
        {
            PCL_ERROR("Couldn't read file target_cloud.pcd \n");
            return;
        }
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(cloudTemp, *target_cloud, mapping);
        // pcl::PointCloud<pcl::PointXYZ>::iterator it = target_cloud->points.begin();
        // while (it != target_cloud->points.end())
        // {
        //     float x, y, z;
        //     x = it->x;
        //     y = it->y;
        //     z = it->z;
        //     if (!pcl_isfinite(x) || !pcl_isfinite(y) || !pcl_isfinite(z))
        //     {
        //         it = target_cloud->points.erase(it);
        //     }
        //     else
        //         ++it;
        // }

        //-------------------------
        pose[0] = -10 * cos(yaw_pre);
        pose[1] = -10 * sin(yaw_pre);
        pose[2] = yaw_pre;
        radianTransform(pose[2]);
        double fitnessScore_thre = 10.0;
        //交叉路口定位
        geometry_msgs::Quaternion geoQuat;
        nav_msgs::Odometry odom;
        odom.header.frame_id = "vehicle_base_link";
        while (ros::ok())
        {
            // 离开交叉路口
            if (!intersectionVerified)
            {
                std::this_thread::sleep_for(std::chrono::seconds(1));
                if (!intersectionVerified)
                {
                    ROS_INFO_STREAM("Leave the vertex  " << path[preVertex_index + 1]);
                    viewer.removeAllPointClouds();
                    break;
                }
            }
            // 定位
            config = loadRegistrationConfig(dataPath);
            //自适应阈值
            config.fitnessScore_thre = std::max(fitnessScore_thre, config.minFitnessScore_thre);
            mtx_visual.lock();
            mtx_cloud.lock();
            bool location_bool = intersectionLocation(pose, target_cloud, lidarCloud, config, viewer);
            mtx_cloud.unlock();
            mtx_visual.unlock();
            // 发布车辆位姿
            if (location_bool)
            {
                geoQuat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pose[2]);
                odom.header.stamp = ros::Time::now();
                odom.pose.pose.orientation.x = geoQuat.x;
                odom.pose.pose.orientation.y = geoQuat.y;
                odom.pose.pose.orientation.z = geoQuat.z;
                odom.pose.pose.orientation.w = geoQuat.w;
                odom.pose.pose.position.x = pose[0];
                odom.pose.pose.position.y = pose[1];
                odom.pose.pose.position.z = 0;
                pubOdom.publish(odom);
 
                double dt = std::pow(10,-6)*(pcl_conversions::toPCL(ros::Time::now()) - lidarCloud->header.stamp);
                ROS_INFO_STREAM("Location time consumption = " << dt <<"s" );

                fitnessScore_thre -= (fitnessScore_thre <= 0) ? 0 : 1;
            }
        }
        return;
    }

    /**
     * Location visualization
     */
    void visual()
    {
        while (ros::ok() && !viewer.wasStopped())
        {
            mtx_visual.lock();
            viewer.spinOnce(10);
            mtx_visual.unlock();
        }
        return;
    }

    /**
     * Navigation
     */
    void navigation()
    {
        while (path.empty())
            std::this_thread::sleep_for(std::chrono::seconds(1));

        std::thread visual_thread(&Navigation::visual, this);
        while (ros::ok())
        {
            if (intersectionVerified)
            {
                if (preVertex_index == path.size() - 2)
                {
                    ROS_INFO("Arrive Destination !!!");
                    break;
                }
                location();
                ++preVertex_index;
            }
        }
        visual_thread.join();
        return;
    }

    void pubIishInfo()
    {
        if (path.empty())
        {
            ROS_INFO_THROTTLE(2, "No Path Received !!!");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else
        {
            std_msgs::Int32 intersectionID;
            intersectionID.data = path[std::min(preVertex_index + 1, static_cast<int>(path.size() - 1))];
            pubIntersectionID.publish(intersectionID);
        }
    }
};

int main(int argc, char **argv)
{
    ios::sync_with_stdio(false);
    ros::init(argc, argv, "navigation");

    Navigation navigation_obj;

    std::thread navigation_thread(&Navigation::navigation, &navigation_obj);
    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();

        navigation_obj.pubIishInfo();

        rate.sleep();
    }

    navigation_thread.join();

    return 0;
}
