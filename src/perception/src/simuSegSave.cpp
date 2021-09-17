
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>                 // 包含kdtree头文件
#include <pcl/kdtree/kdtree_flann.h>           //kdtree搜索
#include <pcl/filters/extract_indices.h>       //按索引提取
#include <pcl/segmentation/extract_clusters.h> //分割聚类

#include <dynamic_reconfigure/server.h>
#include <perception/simuSegSave_Config.h>

pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOrigin;

void topHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI> lidarCloudThis, lidarCloudTrans;
    pcl::fromROSMsg(*msg, lidarCloudThis);
    // 坐标系转换
    tf::TransformListener listener(ros::Duration(1));
    listener.waitForTransform("base_link", "velodyne_top_base_link", ros::Time::now(), ros::Duration(1));
    pcl_ros::transformPointCloud("base_link", lidarCloudThis, lidarCloudTrans, listener);
    lidarCloudTrans.header.frame_id = "base_link";
    *cloudOrigin += lidarCloudTrans;
}

void leftHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI> lidarCloudThis, lidarCloudTrans;
    pcl::fromROSMsg(*msg, lidarCloudThis);
    // 坐标系转换
    tf::TransformListener listener(ros::Duration(1));
    listener.waitForTransform("base_link", "velodyne_left_base_link", ros::Time::now(), ros::Duration(1));
    pcl_ros::transformPointCloud("base_link", lidarCloudThis, lidarCloudTrans, listener);
    lidarCloudTrans.header.frame_id = "base_link";

    *cloudOrigin += lidarCloudTrans;
}

void rightHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PointCloud<pcl::PointXYZI> lidarCloudThis, lidarCloudTrans;
    pcl::fromROSMsg(*msg, lidarCloudThis);
    // 坐标系转换
    tf::TransformListener listener(ros::Duration(1));
    listener.waitForTransform("base_link", "velodyne_right_base_link", ros::Time::now(), ros::Duration(1));
    pcl_ros::transformPointCloud("base_link", lidarCloudThis, lidarCloudTrans, listener);
    lidarCloudTrans.header.frame_id = "base_link";
    *cloudOrigin += lidarCloudTrans;
}

void pointCloudSave(pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
    if (inputCloud->empty())
        return;

    // 坐标系转换
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudTrans(new pcl::PointCloud<pcl::PointXYZI>());
    tf::TransformListener listener(ros::Duration(1.0));
    listener.waitForTransform("vehicle_reference", "base_link", ros::Time::now(), ros::Duration(1.0));
    pcl_ros::transformPointCloud("vehicle_reference", *inputCloud, *inputCloudTrans, listener);
    inputCloudTrans->header.frame_id = "vehicle_reference";

    /*动态参数*/
    int segmentationRadius;
    int node_Id;
    bool save_Bool;
    std::string save_Name;

    ros::param::get("/simuSegSave/segmentation_radius", segmentationRadius);
    ros::param::get("/simuSegSave/node_id", node_Id);
    ros::param::get("/simuSegSave/bool_save", save_Bool);
    ros::param::get("/simuSegSave/save_name", save_Name);
    /*动态参数*/

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCenter(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFinal(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree_1, kdtree_2;
    std::vector<int> index_1, index_2;         //保存每个近邻点的索引
    std::vector<float> distance_1, distance_2; //保存每个近邻点与查找点之间的欧式距离平方

    pcl::PointXYZI zeroPoint(0.f);
    zeroPoint.x = zeroPoint.y = zeroPoint.z = 0;

    // 外围分割
    kdtree_1.setInputCloud(inputCloudTrans); // 设置要搜索的点云，建立KDTree
    pcl::ExtractIndices<pcl::PointXYZI> extract_1;
    if (kdtree_1.radiusSearch(zeroPoint, segmentationRadius, index_1, distance_1) == 0)
    {
        ROS_ERROR("There is no point nearby !!!");
        return;
    }
    extract_1.setInputCloud(inputCloudTrans);
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(index_1);
    extract_1.setIndices(index_ptr);
    extract_1.setNegative(false); //如果设为true,可以提取指定index之外的点云
    extract_1.filter(*cloudCenter);
    // 分割车辆
    kdtree_2.setInputCloud(cloudCenter); // 设置要搜索的点云，建立KDTree
    pcl::ExtractIndices<pcl::PointXYZI> extract_2;
    if (kdtree_2.radiusSearch(zeroPoint, 3.0, index_2, distance_2) == 0)
    {
        ROS_ERROR("There is no vehicle points  !!!");
        return;
    }
    extract_2.setInputCloud(cloudCenter);
    index_ptr = boost::make_shared<std::vector<int>>(index_2);
    extract_2.setIndices(index_ptr);
    extract_2.setNegative(true); //如果设为true,可以提取指定index之外的点云
    extract_2.filter(*cloudFinal);

    if (!save_Bool)
        return;
    // 保存文件
    std::string fileName;
    fileName = "/home/lsj/dev/Mine_WS/simu_data/" + std::to_string(node_Id) + save_Name;
    pcl::io::savePCDFileASCII(fileName, *cloudFinal); //将点云保存到PCD文件中
    ROS_INFO("PCD file saved in  :  [%s]", fileName.c_str());
}

//动态调参
void callback(perception::simuSegSave_Config &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %d %d %s %s",
             config.segmentation_radius,
             config.node_id,
             config.bool_save ? "True" : "False",
             config.save_name.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simuSegSave");
    ros::NodeHandle nh("~");

    /*动态参数调节*/
    dynamic_reconfigure::Server<perception::simuSegSave_Config> server;
    dynamic_reconfigure::Server<perception::simuSegSave_Config>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    /*动态参数调节*/

    std::string lidarTopic_left;
    std::string lidarTopic_right;
    std::string lidarTopic_top;
    nh.param<std::string>("point_topic", lidarTopic_left, "/velodyne_left");
    ROS_INFO("Left lidar topic : %s", lidarTopic_left.c_str());
    nh.param<std::string>("no_ground_point_topic", lidarTopic_right, "/velodyne_right");
    ROS_INFO("Right lidar topic : %s", lidarTopic_right.c_str());
    nh.param<std::string>("ground_point_topic", lidarTopic_top, "/velodyne_top");
    ROS_INFO("Top lidar topic : %s", lidarTopic_top.c_str());
    ROS_INFO("----------------------------------------------------------------------");

    ROS_INFO("\033[1;32m---->\033[0m Simulation Segmentation Save Started.");

    cloudOrigin.reset(new pcl::PointCloud<pcl::PointXYZI>());
    cloudOrigin->header.frame_id = "vehicle_reference";

    ros::Subscriber subLidarCloudLeft = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic_left, 1, &leftHandler);
    ros::Subscriber subLidarCloudRight = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic_right, 1, &rightHandler);
    ros::Subscriber subLidarCloudTop = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic_top, 1, &topHandler);
    ros::Publisher cloudCombined_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud_Combined", 1);

    tf::TransformBroadcaster br;
    tf::Transform transform;

    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();

        transform.setOrigin(tf::Vector3(-2, 0, 1));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "vehicle_reference"));

        pointCloudSave(cloudOrigin);

        cloudOrigin->header.frame_id = "vehicle_reference";
        sensor_msgs::PointCloud2 output;
        toROSMsg(*cloudOrigin, output);
        cloudCombined_pub.publish(output);
        cloudOrigin->clear();

        rate.sleep();
    }
    /***********************读取点云***********************/

    /***********************读取文件***********************/

    /***********************读取文件***********************/

    ROS_INFO("Simulation Segmentation Save");

    return 0;
}