/*此程序由北京理工大学*刘仕杰*编写*/
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_read");
	ros::NodeHandle nh;

    ros::Publisher pubLaserCloudSurroundC;
    ros::Publisher pubLaserCloudSurroundS;
    ros::Publisher pubLaserCloudGround;


    pubLaserCloudSurroundC = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround_corner", 5);
    pubLaserCloudSurroundS = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround_surf", 5);
    pubLaserCloudGround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_ground", 5);

    pcl::PointCloud<pcl::PointXYZI>::Ptr Corner(new pcl::PointCloud<pcl::PointXYZI>);	// Generate pointcloud data，新建指针cloud存放点云
	if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/lsj/煤矿/SURROUND_CLOUD_CORNER.pcd", *Corner) == -1)//*打开点云文件。
	{                                                                           //带路径的格式【注意路径的反斜杠与电脑自己的不同】
		PCL_ERROR("Couldn't read Corner.pcd\n");                             // //不带路径的格式【只是把路径删掉即可】
		return(-1);
	}

    pcl::PointCloud<pcl::PointXYZI>::Ptr Surf(new pcl::PointCloud<pcl::PointXYZI>);	// Generate pointcloud data，新建指针cloud存放点云
	if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/lsj/煤矿/SURROUND_CLOUD_SURF.pcd", *Surf) == -1)//*打开点云文件。
	{                                                                           //带路径的格式【注意路径的反斜杠与电脑自己的不同】
		PCL_ERROR("Couldn't read Surf.pcd\n");                             // //不带路径的格式【只是把路径删掉即可】
		return(-1);
	}

    pcl::PointCloud<pcl::PointXYZI>::Ptr Ground(new pcl::PointCloud<pcl::PointXYZI>);	// Generate pointcloud data，新建指针cloud存放点云
	if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/lsj/煤矿/CLOUD_GROUND.pcd", *Ground) == -1)//*打开点云文件。
	{                                                                           //带路径的格式【注意路径的反斜杠与电脑自己的不同】
		PCL_ERROR("Couldn't read Ground.pcd\n");                             // //不带路径的格式【只是把路径删掉即可】
		return(-1);
	}
        
    ros::Rate rate(1);
    while (ros::ok())
    {
        ros::spinOnce();

        sensor_msgs::PointCloud2 cloudMsgTempC;
        pcl::toROSMsg(*Corner, cloudMsgTempC);
        // cloudMsgTempC.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTempC.header.frame_id = "/camera";
        pubLaserCloudSurroundC.publish(cloudMsgTempC);

        sensor_msgs::PointCloud2 cloudMsgTempS;
        pcl::toROSMsg(*Surf, cloudMsgTempS);
        // cloudMsgTempC.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTempS.header.frame_id = "/camera";
        pubLaserCloudSurroundS.publish(cloudMsgTempS);

        sensor_msgs::PointCloud2 cloudMsgTempG;
        pcl::toROSMsg(*Ground, cloudMsgTempG);
        // cloudMsgTempC.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTempG.header.frame_id = "/camera";
        pubLaserCloudGround.publish(cloudMsgTempG);

        rate.sleep();

    }
    return 0;
}