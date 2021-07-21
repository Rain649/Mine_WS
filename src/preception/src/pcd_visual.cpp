
#include <ros/ros.h>
#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int visualize(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, std::string fileName)
{
  pcl::visualization::CloudViewer viewer(fileName);
  viewer.showCloud(cloud, fileName);

  while (!viewer.wasStopped())
  {
  }

  return 0;
}

int main()
{
  std::string fileName;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
  cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());

  while (true)
  {
    fileName.clear();
    cloud->clear();

    std::cout << "Please Drag The File In:\n"
              << std::endl;
    getline(std::cin, fileName);
    std::cout << "Input Success\n"
              << std::endl;

    fileName = fileName.substr(1, fileName.length() - 3);
    pcl::io::loadPCDFile(fileName, *cloud);
    // pcl::io::loadPCDFile ("/home/lsj/dev/Mine_WS/data/CLOUD_All.pcd", *cloud);
    std::string::size_type iPos = fileName.find_last_of('/') + 1;
    fileName = fileName.substr(iPos, fileName.length() - iPos);

    visualize(cloud, fileName);
  }

  return 0;
}