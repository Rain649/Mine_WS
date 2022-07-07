#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/distances.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>           // 包含kdtree头文件
#include <pcl/kdtree/kdtree_flann.h>     //kdtree搜索
#include <pcl/filters/extract_indices.h> //按索引提取
#include <pcl/filters/passthrough.h>     //直通滤波
#include <pcl/filters/filter.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <yaml-cpp/yaml.h>

#include "intersectionLocation.h"
#include "topoMap.h"
#include "registrationConfig.h"
#include "perception/mineServer.h"

extern int locationTimes;

inline void radianTransform(float &radian)
{
    while (radian > M_PI)
        radian -= 2 * M_PI;
    while (radian <= -M_PI)
        radian += 2 * M_PI;
}

/**
 * Load registration configuration from yaml file
 * @param data_dir pathPlanned of data file
 */
RegistrationConfig loadRegistrationConfig(std::string data_dir)
{
    std::string fin = data_dir + "registrationConfig.yaml";
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
    VehicleState vehicleState;
    std::vector<int> pathPlanned;
    std::string data_dir;
    std_msgs::Bool location_bool;
    int preVertex_index = 0;
    bool intersectionVerified = false;
    bool newLidarCloud_bool = false;
    bool visualize_bool = true;

    TopoMap topoMap;

    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud;
    pcl::visualization::PCLVisualizer::Ptr viewer;

    ///* 车体坐标向局部变换
    tf::TransformBroadcaster tb_local;

    ros::NodeHandle nh;
    ros::Publisher pubIsLocation;
    ros::Publisher pubOdom;
    ros::Publisher pubTarget;
    ros::Publisher pubIntersectionID;
    ros::Publisher pubIntersectionIDindex;
    ros::Publisher pubDistance;

    ros::Subscriber subIntersection;
    ros::Subscriber subLidarCloudCombined;

    ros::Time lastLidarStamp;
    ros::Time currentLidarStamp;
    ros::Time time_deliver;

    // 创建一个client，请求service
    ros::ServiceClient client;

public:
    Navigation() : nh("~")
    {
        //加载参数
        nh.param<std::string>("simu_data_dir", data_dir, "src/common/simu_data/");
        ROS_INFO("simu_data path: %s", data_dir.c_str());

        // 分配内存
        lidarCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

        viewer.reset(new pcl::visualization::PCLVisualizer);
        // 设置点云配准可视化窗口
        viewer->setWindowName("registration");
        viewer->createViewPort(0.0, 0.0, 0.5, 1, v1);
        viewer->setBackgroundColor(0, 0, 0, v1);
        viewer->createViewPort(0.5, 0.0, 1, 1, v2);
        viewer->setBackgroundColor(0.5, 0.5, 0.5, v2);
        viewer->addCoordinateSystem(5);
        viewer->initCameraParameters();
        viewer->setCameraPosition(30, 40, 50, -3, -4, -5, 0);
        viewer->setWindowBorders(false);
        viewer->setSize(400, 260);

        // 加载拓扑地图
        // std::cout << "path : " << getcwd(NULL, 0) << std::endl;
        topoMap = loadMap(data_dir + "Vertex.yaml", data_dir + "Edge.yaml", data_dir);

        // 重新获取节点ID的index
        client = nh.serviceClient<perception::mineServer>("/service/node_ID_Manager");

        // 创建service消息
        perception::mineServer addsrv;
        addsrv.request.mode = 1;
        // 发布service请求，等待应答结果
        while (!client.call(addsrv))
        {
        }

        if (client.call(addsrv))
        {
            pathPlanned = addsrv.response.pathPlanned;
            printf("Path: ");
            for (auto i : pathPlanned)
                printf("%d -> ", i);
            printf("\n");
        }
        else
            ROS_ERROR("Failed to call service node_ID_Manager");

        addsrv.request.mode = 2;
        if (client.call(addsrv))
        {
            preVertex_index = addsrv.response.nodeID_Index;
            ROS_INFO("Current Node ID: %d", pathPlanned[preVertex_index]);
            if (preVertex_index > 0)
                --preVertex_index;
        }
        else
            ROS_ERROR("Failed to call service node_ID_Manager");

        // 定义topic收发
        pubIsLocation = nh.advertise<std_msgs::Bool>("isLocation", 1);
        pubOdom = nh.advertise<nav_msgs::Odometry>("intersectionOdom", 1);
        pubTarget = nh.advertise<nav_msgs::Odometry>("targetPoint", 1);
        pubIntersectionID = nh.advertise<std_msgs::Int32>("intersection_id", 1);
        pubIntersectionIDindex = nh.advertise<std_msgs::Int32>("intersectionID_id", 1);
        pubDistance = nh.advertise<std_msgs::Float32>("distance", 1);

        subIntersection = nh.subscribe<std_msgs::Bool>("/intersectionDetection/intersectionVerified", 1, &Navigation::intersectionHandler, this);
        subLidarCloudCombined = nh.subscribe<sensor_msgs::PointCloud2ConstPtr>("/lidarCloudProcess/cloud_Combined", 1, &Navigation::cloudHandler, this);
    }

    void intersectionHandler(const std_msgs::Bool msg)
    {
        intersectionVerified = msg.data;
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr msg)
    {
        lastLidarStamp = currentLidarStamp;
        currentLidarStamp = msg->header.stamp;
        // time_deliver = ros::Time::now();
        // ROS_ERROR("DELIVER FROM LIDAR TO NAVIGATION PROGRAM TIME COST: %f s", (time_deliver - currentLidarStamp).toSec());

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCombined(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromROSMsg(*msg, *cloudCombined);

        if (cloudCombined->empty())
            return;

        std::vector<int> index;      //保存每个近邻点的索引
        std::vector<float> distance; //保存每个近邻点与查找点之间的欧式距离平方
        // 外围分割
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(cloudCombined); // 设置要搜索的点云，建立KDTree
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointXYZ zeroPoint;
        zeroPoint.x = zeroPoint.y = zeroPoint.z = 0;
        int segmentationRadius = 25;
        if (kdtree.radiusSearch(zeroPoint, segmentationRadius, index, distance) == 0)
        {
            ROS_ERROR("There is no point nearby !!!");
            return;
        }
        extract.setInputCloud(cloudCombined);
        boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index);
        extract.setIndices(index_ptr);
        extract.setNegative(false); //如果设为true,可以提取指定index之外的点云
        mtx_cloud.lock();
        lidarCloud->clear();
        extract.filter(*lidarCloud);
        lidarCloud->header.stamp = cloudCombined->header.stamp;
        newLidarCloud_bool = true;
        mtx_cloud.unlock();

        return;
    }

    /**
     * Location at intersection
     */
    bool location()
    {
        // 加载目标点云，获取点云配准先验位姿
        float yaw_pre;
        std::vector<double> target_point = topoMap.get_targetPoint(pathPlanned[preVertex_index + 1], pathPlanned[preVertex_index + 2]);
        std::array<double, 3> startPoint = topoMap.get_startPoint(pathPlanned[preVertex_index + 1], pathPlanned[preVertex_index]);
        float x_target = target_point[0];
        float y_target = target_point[1];
        float yaw_target = target_point[2] * M_PI / 180;
        radianTransform(yaw_target);
        std::string fileName;
        if (pathPlanned[preVertex_index] == 0)
        {
            yaw_pre = 0;
        }
        else
        {
            yaw_pre = topoMap.get_angleDiff(pathPlanned[preVertex_index], pathPlanned[preVertex_index + 1]);
        }
        fileName = data_dir + std::to_string(pathPlanned[preVertex_index + 1]) + "_node.pcd";
        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        target_cloud->header.frame_id = "vehicle_base_link";
        pcl::PointCloud<pcl::PointXYZ> cloudTemp;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName, cloudTemp) == -1)
        {
            PCL_ERROR("Couldn't read file target_cloud.pcd \n");
            return false;
        }
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(cloudTemp, *target_cloud, mapping);
        //-------------------------
        vehicleState.x = startPoint[0];
        vehicleState.y = startPoint[1];
        vehicleState.yaw = yaw_pre;
        // vehicleState.x = -10 * cos(yaw_pre);
        // vehicleState.y = -10 * sin(yaw_pre);
        // vehicleState.yaw = yaw_pre;
        radianTransform(vehicleState.yaw);
        //交叉路口定位
        geometry_msgs::Quaternion geoQuat;
        nav_msgs::Odometry odom;
        geometry_msgs::Quaternion geoQuat_localMap;
        geometry_msgs::Quaternion geoQuat_target;
        nav_msgs::Odometry odom_target;
        odom.header.frame_id = "vehicle_base_link";
        odom.header.stamp = ros::Time::now();
        odom_target.header.frame_id = "localMap";
        odom_target.header.stamp = ros::Time::now();

        //配准参数
        config = loadRegistrationConfig(data_dir);
        double fitnessScore_thre = 10.0;

        float distance{100};
        while (ros::ok())
        {
            // 离开交叉路口
            if (!intersectionVerified && distance <= 6)
            {
                ros::Time time_start = ros::Time::now();
                while (!intersectionVerified)
                {
                    if ((ros::Time::now() - time_start).toSec() > 0.5)
                    {
                        ROS_ERROR_STREAM("TimeOut Leave The Vertex  " << pathPlanned[preVertex_index + 1]);
                        mtx_visual.lock();
                        viewer->removeAllPointClouds();
                        mtx_visual.unlock();
                        locationTimes = 0;
                        return true;
                    }
                }
            }
            else if (!intersectionVerified && distance > 20.0)
            {
                ROS_ERROR("MISTAKE");
                mtx_visual.lock();
                viewer->removeAllPointClouds();
                mtx_visual.unlock();
                locationTimes = 0;
                return false;
            }
            // 收到新点云定位
            if (newLidarCloud_bool)
            {
                newLidarCloud_bool = false;
                // 自适应阈值
                config.fitnessScore_thre = std::max(fitnessScore_thre, config.minFitnessScore_thre);

                mtx_visual.lock();
                mtx_cloud.lock();
                // 配准定位
                if (visualize_bool)
                    location_bool.data = intersectionLocation(vehicleState, target_cloud, lidarCloud, config, viewer);
                else
                    location_bool.data = intersectionLocation(vehicleState, target_cloud, lidarCloud, config, nullptr);

                mtx_cloud.unlock();
                mtx_visual.unlock();

                if (location_bool.data)
                {
                    ROS_ERROR_COND(locationTimes == 1, "Enter the vertex %d ", pathPlanned[preVertex_index + 1]);
                    fitnessScore_thre -= (fitnessScore_thre <= 0) ? 0 : 1;

                    // ros::Time time_process = ros::Time::now();
                    // ROS_ERROR("LOCATION PROCESS TIME COST: %f s", (time_process - time_deliver).toSec());
                }
                else
                {
                    // vehicleState.x += vehicleState.v * cos(vehicleState.yaw) * ((currentLidarStamp - lastLidarStamp).toSec());
                    // vehicleState.y += vehicleState.v * sin(vehicleState.yaw) * ((currentLidarStamp - lastLidarStamp).toSec());
                    // ROS_INFO("Position Interpolation");
                }
                pubIsLocation.publish(location_bool);

                // 发布车辆位姿
                geoQuat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, vehicleState.yaw);
                odom.header.stamp = ros::Time::now();
                odom.pose.pose.orientation.x = geoQuat.x;
                odom.pose.pose.orientation.y = geoQuat.y;
                odom.pose.pose.orientation.z = geoQuat.z;
                odom.pose.pose.orientation.w = geoQuat.w;
                odom.pose.pose.position.x = vehicleState.x;
                odom.pose.pose.position.y = vehicleState.y;
                odom.pose.pose.position.z = 0;
                odom.twist.twist.linear.x = vehicleState.v * cos(vehicleState.yaw);
                odom.twist.twist.linear.y = vehicleState.v * sin(vehicleState.yaw);
                odom.twist.twist.linear.y = 0;
                pubOdom.publish(odom);

                // 发布局部地图坐标变换
                tf::Transform transform;
                float x_temp = -vehicleState.x * cos(vehicleState.yaw) - vehicleState.y * sin(vehicleState.yaw);
                float y_temp = vehicleState.x * sin(vehicleState.yaw) - vehicleState.y * cos(vehicleState.yaw);
                transform.setOrigin(tf::Vector3(x_temp, y_temp, 0));
                geometry_msgs::Quaternion geoQuat_inv = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -vehicleState.yaw);
                transform.setRotation(tf::Quaternion(geoQuat_inv.x, geoQuat_inv.y, geoQuat_inv.z, geoQuat_inv.w));
                tf::StampedTransform st(transform, ros::Time::now(), "vehicle_base_link", "localMap");
                tb_local.sendTransform(st);

                // 发布目标点位姿
                geoQuat_target = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_target);
                odom_target.header.stamp = ros::Time::now();
                odom_target.pose.pose.orientation.x = geoQuat_target.x;
                odom_target.pose.pose.orientation.y = geoQuat_target.y;
                odom_target.pose.pose.orientation.z = geoQuat_target.z;
                odom_target.pose.pose.orientation.w = geoQuat_target.w;
                odom_target.pose.pose.position.x = x_target;
                odom_target.pose.pose.position.y = y_target;
                odom_target.pose.pose.position.z = 0;
                pubTarget.publish(odom_target);

                // 离开交叉路口
                pcl::PointXY p = {vehicleState.x, vehicleState.y};
                pcl::PointXY t = {x_target, y_target};
                float yaw_diff = abs(vehicleState.yaw - yaw_target);
                if (yaw_diff < config.yaw_thre * M_PI / 180 || 2 * M_PI - yaw_diff < config.yaw_thre * M_PI / 180)
                {
                    distance = pcl::euclideanDistance(p, t) * abs(cos(yaw_diff));
                    std_msgs::Float32 distance_f32;
                    distance_f32.data = distance;
                    pubDistance.publish(distance_f32);
                    if (distance < 3)
                    {
                        ROS_ERROR_STREAM("Arrive Destination Leave The Vertex  " << pathPlanned[preVertex_index + 1]);
                        mtx_visual.lock();
                        viewer->removeAllPointClouds();
                        mtx_visual.unlock();
                        locationTimes = 0;
                        break;
                    }
                }
                else
                {
                    // ROS_INFO("target yaw: %f ; actual yaw : %f ; yaw error : %f", yaw_target * 180 / M_PI, vehicleState.yaw * 180 / M_PI, yaw_diff * 180 / M_PI);
                }
            }
        }
        return true;
    }

    /**
     * Location_ isualization_
     */
    void visual()
    {
        while (ros::ok() && !viewer->wasStopped())
        {
            mtx_visual.lock();
            viewer->spinOnce(10);
            mtx_visual.unlock();
        }
        return;
    }

    /**
     * Navigation
     */
    void navigation()
    {
        while (pathPlanned.empty())
            std::this_thread::sleep_for(std::chrono::seconds(1));
        if (visualize_bool)
        {
            std::thread visual_thread(&Navigation::visual, this);
            while (ros::ok())
            {
                if (intersectionVerified)
                {
                    if (static_cast<size_t>(preVertex_index) == pathPlanned.size() - 2)
                    {
                        ROS_INFO("Arrive Destination !!!");
                        break;
                    }
                    if (location())
                    {
                        ++preVertex_index;
                        // 创建service消息
                        perception::mineServer addsrv;
                        addsrv.request.mode = 3;
                        addsrv.request.nodeID_Index = preVertex_index + 1;
                        // 发布service请求，等待应答结果
                        if (client.call(addsrv))
                        {
                            // ROS_INFO("nodeID_index: %d", addsrv.response.nodeID_Index);
                        }
                        else
                            ROS_ERROR("Failed to call service node_ID_Manager");
                    }
                    location_bool.data = false;
                    pubIsLocation.publish(location_bool);
                }
            }
            visual_thread.join();
        }
        else
        {
            while (ros::ok())
            {
                ROS_ERROR_ONCE("Enter the vertex %d ", pathPlanned[preVertex_index + 1]);
                if (intersectionVerified)
                {
                    if (static_cast<size_t>(preVertex_index) == pathPlanned.size() - 2)
                    {
                        ROS_INFO("Arrive Destination !!!");
                        break;
                    }
                    if (location())
                    {
                        ++preVertex_index;
                    }
                    location_bool.data = false;
                    pubIsLocation.publish(location_bool);
                }
            }
        }
        return;
    }

    void pubIishInfo()
    {
        if (pathPlanned.empty())
        {
            ROS_INFO_THROTTLE(2, "No Path Received !!!");
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        else
        {
            std_msgs::Int32 intersectionID, intersectionIDindex;
            intersectionIDindex.data = std::min(preVertex_index + 1, static_cast<int>(pathPlanned.size() - 1));
            intersectionID.data = pathPlanned[intersectionIDindex.data];
            pubIntersectionID.publish(intersectionID);
            pubIntersectionIDindex.publish(intersectionIDindex);
        }
    }
};

int main(int argc, char **argv)
{
    ios::sync_with_stdio(false);
    ros::init(argc, argv, "navigation");

    Navigation navigation_obj;

    std::thread navigation_thread(&Navigation::navigation, &navigation_obj);
    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        navigation_obj.pubIishInfo();

        rate.sleep();
    }

    navigation_thread.join();

    return 0;
}
