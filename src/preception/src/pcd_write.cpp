#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
 
using namespace std;  // 可以加入 std 的命名空间

	 
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurf;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCorner;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudGround;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCAS;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudAll;
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudThis;

void CornerHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	laserCloudCorner -> clear();
	// if( sizeof(msg) == 0 ) return;
    pcl::fromROSMsg(*msg, *laserCloudCorner);
	pcl::io::savePCDFileASCII("/home/lsj/dev/Mine_WS/data/SURROUND_CLOUD_CORNER.pcd", *laserCloudCorner); //将点云保存到PCD文件中
	ROS_INFO("Corner Save");
}

void SurfHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	laserCloudSurf -> clear();
    pcl::fromROSMsg(*msg, *laserCloudSurf);
	pcl::io::savePCDFileASCII("/home/lsj/dev/Mine_WS/data/SURROUND_CLOUD_SURF.pcd", *laserCloudSurf); //将点云保存到PCD文件中
	ROS_INFO("Surf Save");
}

void GroundHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	laserCloudGround -> clear();
    pcl::fromROSMsg(*msg, *laserCloudGround);
	pcl::io::savePCDFileASCII("/home/lsj/dev/Mine_WS/data/CLOUD_GROUND.pcd", *laserCloudGround); //将点云保存到PCD文件中
	ROS_INFO("Ground Save");
}

void CASHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	laserCloudCAS-> clear();
    pcl::fromROSMsg(*msg, *laserCloudCAS);
	pcl::io::savePCDFileASCII("/home/lsj/dev/Mine_WS/data/CLOUD_CAS.pcd", *laserCloudCAS); //将点云保存到PCD文件中
	ROS_INFO("CAS Save");
}

void AllHandler(const sensor_msgs::PointCloud2ConstPtr& msg)
{
	laserCloudThis-> clear();
    pcl::fromROSMsg(*msg, *laserCloudThis);
	*laserCloudAll += *laserCloudThis;
	ROS_INFO("Get Data");

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_write");
	ros::NodeHandle nh;

	laserCloudSurf.reset(new pcl::PointCloud<pcl::PointXYZI>());
	laserCloudCorner.reset(new pcl::PointCloud<pcl::PointXYZI>());
	laserCloudGround.reset(new pcl::PointCloud<pcl::PointXYZI>());
	laserCloudCAS.reset(new pcl::PointCloud<pcl::PointXYZI>());
	laserCloudAll.reset(new pcl::PointCloud<pcl::PointXYZI>());
	laserCloudThis.reset(new pcl::PointCloud<pcl::PointXYZI>());

	ros::Subscriber subLaserCloudCorner;
	ros::Subscriber subLaserCloudSurf;
	ros::Subscriber subLaserCloudGround;
	ros::Subscriber subLaserCloudCAS;
	ros::Subscriber subLaserCloudAll;

	// subLaserCloudCorner = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround_corner", 5, &CornerHandler);
	// subLaserCloudSurf = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround_surf", 5, &SurfHandler);
	// subLaserCloudGround = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_ground", 5, &GroundHandler);
	// subLaserCloudCAS = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround_CAS", 5, &CASHandler);
	
	subLaserCloudCorner = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround_corner", 5, &AllHandler);
	subLaserCloudSurf = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surround_surf", 5, &AllHandler);
	subLaserCloudGround = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_ground", 5, &AllHandler);
	
    ros::Rate rate(1);
    while (ros::ok())
    {
        ros::spinOnce();

		ROS_INFO("Waiting");

        rate.sleep();
    }
	
	pcl::io::savePCDFileASCII("/home/lsj/dev/Mine_WS/data/CLOUD_All.pcd", *laserCloudAll); //将点云保存到PCD文件中
	ROS_INFO("All Save");

    return 0;
}
