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
#include <pcl/common/transforms.h>	//	pcl::transformPointCloud 用到这个头文件
#include <pcl/filters/extract_indices.h>    //按索引提取


#include <vector>
#include <string>

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudAll;
pcl::PointCloud<pcl::PointXYZI>::Ptr keyPoint_Neighbor;

pcl::PointCloud<pcl::PointXYZI>::Ptr transform2zero(pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud, pcl::PointXYZI zeroPoint)
{

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << -zeroPoint.x, -zeroPoint.y, -zeroPoint.z;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZI> ());
  pcl::transformPointCloud(*inputCloud, *transformed_cloud, transform);
  return transformed_cloud;
}

void segmentation_1(int index, pcl::PointXYZI thisKeyPoint, int segmentationRadius)
{
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(laserCloudAll); // 设置要搜索的点云，建立KDTree
    std::vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
	std::vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
    
    if (kdtree.radiusSearch(thisKeyPoint, segmentationRadius+5, pointIdxRadiusSearch, pointRadiusSquaredDistance)==0)
    {
        return;
    }

    for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
    {
        keyPoint_Neighbor->push_back(laserCloudAll->points[pointIdxRadiusSearch[i]]);
    }
    keyPoint_Neighbor = transform2zero(keyPoint_Neighbor, thisKeyPoint);

    std::string fileName;
    fileName = "/home/lsj/dev/Mine_WS/data/"+std::to_string(index)+"_whole.pcd";
    pcl::io::savePCDFileASCII(fileName, *keyPoint_Neighbor); //将点云保存到PCD文件中
    ROS_INFO("Whole PCD saved in  :  [%s]", fileName.c_str());

    ROS_INFO("====================================================");
    return;
}

void segmentation_2(int index, int segmentationRadius)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_0(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_1(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_2(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_3(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_4(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster_5(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCenter(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudIntersections(new pcl::PointCloud<pcl::PointXYZI>());

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
	kdtree.setInputCloud(keyPoint_Neighbor); // 设置要搜索的点云，建立KDTree
    std::vector<int> pointIdxRadiusSearch;  //保存每个近邻点的索引
	std::vector<float> pointRadiusSquaredDistance;  //保存每个近邻点与查找点之间的欧式距离平方
    
    pcl::PointXYZI thisKeyPoint(0.f);
    thisKeyPoint.x = thisKeyPoint.y = thisKeyPoint.z = 0;
    if (kdtree.radiusSearch(thisKeyPoint, segmentationRadius, pointIdxRadiusSearch, pointRadiusSquaredDistance)==0)
    {   
        ROS_ERROR("There is no point nearby !!!");
        return;
    }

    //**分割点云**//
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(keyPoint_Neighbor);
    boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(pointIdxRadiusSearch);
    extract.setIndices(index_ptr);
    extract.setNegative(false);//如果设为true,可以提取指定index之外的点云
    extract.filter(*cloudCenter);
    // pcl::copyPointCloud(*keyPoint_Neighbor, pointIdxRadiusSearch, *cloudCenter);//将对应索引的点存储
    ROS_INFO_STREAM("Extract Center Success !!!    Center Number = " << cloudCenter->size());
    extract.setNegative(true);//提取指定index之外的点云
    extract.filter(*cloudIntersections);
    ROS_INFO_STREAM("Extract Intersection Success !!!    Intersection Number = " << cloudIntersections->size());

    cloudCenter = transform2zero(cloudCenter, thisKeyPoint);
    cloudIntersections = transform2zero(cloudIntersections, thisKeyPoint);
    ROS_DEBUG("Transform Position Success !!!");

    std::string fileName;
    /*保存交叉路口中心点云*/
    fileName = "/home/lsj/dev/Mine_WS/data/"+std::to_string(index)+"_center.pcd";
    try
    {
        pcl::io::savePCDFileASCII(fileName, *cloudCenter); //将点云保存到PCD文件中
        ROS_INFO("\033[1;32m---->\033[0m Center PCD saved in  :  [%s]", fileName.c_str());
    }catch(...)
    {
        ROS_ERROR("There is no point in Center !!!");
    }
    /*保存交叉路口岔路点云*/
    fileName = "/home/lsj/dev/Mine_WS/data/"+std::to_string(index)+"_intersection.pcd";
    try
    {
        pcl::io::savePCDFileASCII(fileName, *cloudIntersections); //将点云保存到PCD文件中
        ROS_INFO("\033[1;32m---->\033[0m Intersection PCD saved in  :  [%s]", fileName.c_str());
    }catch(...)
    {
        ROS_ERROR("There is no point in Center !!!");
    }

    ROS_INFO("====================================================");
    return;
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
    ROS_INFO("\033[1;32m---->\033[0m Please Drag The PCD File In:");
    std::getline(std::cin,fileName);
    if(fileName=="")
        fileName = "_/home/lsj/dev/Mine_WS/data/CLOUD_All.pcd__";
    laserCloudAll.reset(new pcl::PointCloud<pcl::PointXYZI>);
    keyPoint_Neighbor.reset(new pcl::PointCloud<pcl::PointXYZI>);
	while(pcl::io::loadPCDFile<pcl::PointXYZI>(fileName.substr(1, fileName.length() - 3), *laserCloudAll) == -1)//*打开点云文件。
	{                                                                           //带路径的格式【注意路径的反斜杠与电脑自己的不同】
		PCL_ERROR("Couldn't read laserCloudAll.pcd : [%s] ", (fileName.substr(1, fileName.length() - 3)).c_str());
        ROS_INFO("\033[1;32m---->\033[0m Please Drag The PCD File In:");
        std::getline(std::cin,fileName);
        if(fileName=="") fileName = "_/home/lsj/dev/Mine_WS/data/CLOUD_All.pcd__";
	}
    ROS_INFO_STREAM("The Number of Points of This PCD File is :  " << laserCloudAll->size());
/***********************读取点云***********************/

/***********************读取文件***********************/
    std::ifstream infile;  //infile是一个文件流，因此其实还是对流进行的操作
    fileName.clear();
    ROS_INFO("Please Drag The Intersection Information File In:");
    std::getline(std::cin,fileName);
    if(fileName=="") fileName = "_/home/lsj/dev/Mine_WS/data/intersection_info.txt__";
    infile.open(fileName.substr(1, fileName.length() - 3));
    while(!infile) //判断是否存在ifstream infile
    {
        ROS_ERROR("No such File : [%s] ", (fileName.substr(1, fileName.length() - 3)).c_str());
        ROS_INFO("\033[1;32m---->\033[0m Please Drag The Intersection Information File In:");
        std::getline(std::cin,fileName);
        if(fileName=="") fileName = "_/home/lsj/dev/Mine_WS/data/intersection_info.txt__";
        infile.open(fileName.substr(1, fileName.length() - 3));
    }
    ROS_INFO("\033[1;32m---->\033[0m Read File Successfully");

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
                keyPoint_Neighbor->clear();

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
                segmentation_1(index, thisKeyPoint, segmentationRadius);
                segmentation_2(index, segmentationRadius);

                break;
        }


    }
/***********************数据处理***********************/

    infile.close();

	ROS_INFO("Segmentation Save");

    return 0;
}