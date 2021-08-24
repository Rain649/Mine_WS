#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "intersectionLocation.h"

int main()
{
    /********读取数据********/
    std::string fin = "/home/lsj/dev/Mine_WS/src/perception/include/ndtData.yaml";
    YAML::Node config = YAML::LoadFile(fin);
    float yaw = config["yaw"].as<float>() * M_PI / 180;
    float x = config["x"].as<float>();
    float y = config["y"].as<float>();
    std::string test1File = config["test1File"].as<std::string>();
    std::string test2File = config["test2File"].as<std::string>();
    static float yaw_pre = config["yaw_pre"].as<float>() * M_PI / 180;
    static float x_pre = config["x_pre"].as<float>();
    static float y_pre = config["y_pre"].as<float>();
    /********读取数据********/
    std::vector<float> pose{x_pre, y_pre, yaw_pre};

    //加载目标点云pcd
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(test1File, *target_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test1.pcd \n");
        return (-1);
    }
    std::cout << "Loaded " << target_cloud->size() << " data points from room_scan1.pcd" << std::endl;
    //加载需要配准点云pcd
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(test2File, *input_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file test2.pcd\n");
        return (-1);
    }
    std::cout << "Loaded " << input_cloud->size() << " data points from room_scan2.pcd" << std::endl;

    // static pcl::PointXYZ thisP;
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud_Trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud_Trans(new pcl::PointCloud<pcl::PointXYZ>);
    // // range filter,去除周围点云
    // range_filter(*input_cloud,*inputCloud_Trans,4,0.1);
    // range_filter(*target_cloud,*targetCloud_Trans,4,0.1);

    for (const auto &p : *input_cloud)
    {
        pcl::PointXYZ thisP;
        float x1 = p.z + x;
        float y1 = p.x + y;
        float z1 = p.y;

        thisP.x = x1 * cos(yaw) + y1 * sin(yaw);
        thisP.y = -x1 * sin(yaw) + y1 * cos(yaw);
        thisP.z = z1;

        inputCloud_Trans->push_back(thisP);
    }
    for (const auto &p : *target_cloud)
    {
        pcl::PointXYZ thisP;
        float x1 = p.z;
        float y1 = p.x;
        float z1 = p.y;

        thisP.x = x1;
        thisP.y = y1;
        thisP.z = z1;

        targetCloud_Trans->push_back(thisP);
    }

    pcl::visualization::PCLVisualizer viewer("ndt");
    clock_t startTime, endTime;
    startTime = clock(); //计时开始
    intersectionLocation(pose, target_cloud, input_cloud, viewer);
    endTime = clock(); //计时结束
    std::cout << "The run time is: " << (int)(endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(10);
        boost::this_thread::sleep(boost::posix_time::seconds(0.5));
    }
    return 0;
}