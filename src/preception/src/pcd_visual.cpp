#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

int main()
 {
//    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOri (new pcl::PointCloud<pcl::PointXYZI>);
//    pcl::io::loadPCDFile ("/home/lsj/煤矿/CLOUD_CAS.pcd", *cloudOri);
//    pcl::visualization::CloudViewer viewerOri ("Original Cloud Viewer");
//    viewerOri.showCloud (cloudOri);

   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
   pcl::io::loadPCDFile ("/home/lsj/dev/Mine_WS/data/Test.pcd", *cloud);
   pcl::visualization::CloudViewer viewer ("Cloud Viewer");
   viewer.showCloud (cloud);
   while (!viewer.wasStopped ())
   {
   }
   return 0;
 }