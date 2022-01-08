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
#include <pcl/filters/passthrough.h>           //直通滤波
#include <pcl/filters/conditional_removal.h>   //条件滤波

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointCloudProcess");
    ros::NodeHandle nh("~");

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::string simu_path = "/home/lsj/dev/Mine_WS/src/perception/simu_data/";
    for (int i = 1; i <= 18; ++i)
    {
        inputCloud->clear();
        outputCloud->clear();
        std::string fileName = std::to_string(i) + "_node.pcd";
        std::string file_path = simu_path + fileName;
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *inputCloud) == -1)
        {
            PCL_ERROR("Couldn't read file inputCloud.pcd \n");
        }
        //分离地面点
        pcl::PassThrough<pcl::PointXYZ> groundFilter;
        groundFilter.setInputCloud(inputCloud);
        groundFilter.setFilterFieldName("z");
        groundFilter.setFilterLimits(-0.8, 2);
        groundFilter.setFilterLimitsNegative(false);
        groundFilter.filter(*outputCloud);

        // 保存文件
        std::string finalFileName= "/home/lsj/dev/Mine_WS/src/perception/simu_data_New/" + fileName;
        pcl::io::savePCDFileASCII(finalFileName, *outputCloud); //将点云保存到PCD文件中
        ROS_INFO("PCD file saved in  :  [%s]", finalFileName.c_str());
    }

    return 0;
}