#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>
#include <string> 

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h> // 包含kdtree头文件
#include <pcl/kdtree/kdtree_flann.h>    //kdtree搜索


#include <vector>
#include <string>

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudAll;
std::vector<pcl::PointCloud<pcl::PointXYZI>> segCloud_Vec;

void segmentation(int index, pcl::PointXYZI thisKeyPoint, int cluster_Num, int segmentationRadius)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_0(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_1(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_3(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_4(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_5(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudAfterCondition(new pcl::PointCloud<pcl::PointXYZI>());

    // pcl::ConditionOr<pcl::PointXYZI>::Ptr range_Condition(new pcl::ConditionOr<pcl::PointXYZI>());  //条件滤波
    // pcl::Filter<pcl::PointCloud>:: afterFilter;
    // pcl::ConditionalRemoval<pcl::PointXYZI> condition(true);
    // condition.setCondition(range_Condition);
    // condition.setInputCloud(laserCloudAll);
    // condition.setKeepOrganized(false);
    // condition.filter(*cloudAfterCondition);

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(laserCloudAll); // 设置要搜索的点云，建立KDTree
    std::vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
	std::vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
    pcl::PointCloud<pcl::PointXYZI>::Ptr keyPoint_Neighbor(new pcl::PointCloud<pcl::PointXYZI>());
    
    if (kdtree.radiusSearch(thisKeyPoint, segmentationRadius+5, pointIdxRadiusSearch, pointRadiusSquaredDistance)==0)
    {
        return;
    }

    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    {
        keyPoint_Neighbor->push_back(laserCloudAll->points[pointIdxRadiusSearch[i]]);
    }

    std::string fileName;
    fileName = "/home/lsj/dev/Mine_WS/data/"+std::to_string(index)+".pcd";
    pcl::io::savePCDFileASCII(fileName, *keyPoint_Neighbor); //将点云保存到PCD文件中
    ROS_INFO("[%s]", fileName.c_str());
    
    std::vector<pcl::PointCloud<pcl::PointXYZI>> result;
    result.push_back(*keyPoint_Neighbor);


    ROS_INFO("====================================================");
    
}

void save()
{

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "segmentationSave");
	ros::NodeHandle nh;

    ROS_INFO("\033[1;32m---->\033[0m Segmentation Save Started.");

/***********************读取点云***********************/
    std::string fileName;
    ROS_INFO("Please Drag The PCD File In:");
    getline(std::cin,fileName);
    laserCloudAll.reset(new pcl::PointCloud<pcl::PointXYZI>);
	if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/lsj/dev/Mine_WS/data/CLOUD_All.pcd", *laserCloudAll) == -1)//*打开点云文件。
	{                                                                           //带路径的格式【注意路径的反斜杠与电脑自己的不同】
		PCL_ERROR("Couldn't read laserCloudAll.pcd\n");                             // //不带路径的格式【只是把路径删掉即可】
		return(-1);
	}
    else ROS_INFO_STREAM("The Number of Points of This PCD File is :  " << laserCloudAll->size());
/***********************读取点云***********************/

/***********************读取文件***********************/
    std::ifstream infile;  //infile是一个文件流，因此其实还是对流进行的操作
    fileName.clear();
    ROS_INFO("Please Drag The Intersection Information File In:");
    getline(std::cin,fileName);
    // std::cout << fileName.substr(1, fileName.length() - 3) << std::endl;
    infile.open(fileName.substr(1, fileName.length() - 3));
    if (!infile) //判断是否存在ifstream infile
    {
        ROS_ERROR("-----------No such File-----------");
    }
    else
    {
        ROS_INFO("-----------Read File Successfully-----------");
    }

    std::vector<float> data_Vec;
    if (infile.is_open()) //判断文件流是否处于打开状态
    {   
        std::string firstLine;
        std::getline(infile, firstLine);
        while (infile.good() && !infile.eof())
        {
            float temp;
            infile >> temp;
            data_Vec.push_back(temp); //将数据读入到data_vector
        }
    }
/***********************读取文件***********************/

/***********************数据处理***********************/
    int index;
    pcl::PointXYZI thisKeyPoint;
    int clusterNum;
    int peakNum;
    int segmentationRadius;
    int i = 0;
    for (std::vector<float>::iterator iter = data_Vec.begin(); iter != data_Vec.end(); ++iter)
    {   
        ++i;
        switch(i%7)
        {
            case 1:
                index = *iter;
                segCloud_Vec.clear();

                break;
            case 2:
                thisKeyPoint.x = *iter;

                break;
            case 3:
                thisKeyPoint.y = *iter;

                break;
            case 4:
                thisKeyPoint.z = *iter;

                break;
            case 5:
                clusterNum = *iter;

                break;
            case 6:
                peakNum = *iter;

                break;
            case 0:
                segmentationRadius = *iter;
                segmentation(index, thisKeyPoint, clusterNum, segmentationRadius);

                break;
        }


    }
/***********************数据处理***********************/

    infile.close();

	ROS_INFO("Segmentation Save");

    return 0;
}