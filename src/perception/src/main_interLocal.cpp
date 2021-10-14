#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>

#include "intersectionLocation.h"

std::mutex mtx;

void visual(pcl::visualization::PCLVisualizer &viewer)
{
    while (!viewer.wasStopped())
    {
        mtx.lock();
        viewer.spinOnce(10);
        mtx.unlock();
    }
    return;
}

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
    float yaw_pre = config["yaw_pre"].as<float>() * M_PI / 180;
    float x_pre = config["x_pre"].as<float>();
    float y_pre = config["y_pre"].as<float>();
    /********读取数据********/
    ///* input->target的x, y, yaw
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

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud_Trans(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr targetCloud_Trans(new pcl::PointCloud<pcl::PointXYZ>);

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
    std::thread visual_thread(visual, std::ref(viewer));
    clock_t startTime, endTime;
    startTime = clock(); //计时开始
    while (1)
    {
        mtx.lock();
        viewer.removeAllPointClouds();
        intersectionLocation(pose, targetCloud_Trans, inputCloud_Trans, viewer);
        mtx.unlock();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    endTime = clock(); //计时结束
    std::cout << "The run time is: " << (int)(endTime - startTime) / CLOCKS_PER_SEC << " s" << std::endl;

    visual_thread.join();

    return 0;
}