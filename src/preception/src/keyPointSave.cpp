
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>

#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include <vector>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <perception/param_Config.h>

#include <utility.h>

class keyPointSave
{
private:
    short int clusterNum = 0;
    short int peakNum = 0;
    short int segmentationRadius = 0;
    short int clusterNum_Pre = -1;
    short int peakNum_Pre = -1;
    short int counter = 0;
    short int clusterNum_max = 0;
    short int peakNum_max = 0;

    double peakDistance_min = 20;
    short int segmentationRadius_max = 0;
    short int clusterNum_final;
    short int peakNum_final;
    double peakDistanceSum_final;

    double interval;
    double peakDistance_Max;

    bool intersectionVerified = false;
    bool record_Bool = false;
    bool save_Bool = false;

    std::string save_Name;
    std::vector<short int> radius_Vec;
    std::vector<std::pair<short int, short int>> thisNum_Vec;
    std::vector<std::pair<pcl::PointXYZ, short int>> currentPosition_Vec;
    std::vector<std::pair<short int, pcl::PointXYZ>> keyPoints_Vec;
    std::vector<std::pair<short int, short int>> numOfIntersection_Vec;

    pcl::PointXYZ thisKeyPoint;
    pcl::PointXYZ currentPosition;

    ros::NodeHandle nh;
    ros::Time begin;
    ros::Subscriber subPeakNum;
    ros::Subscriber subClusterNum;
    ros::Subscriber subIntersectionVerified;
    ros::Subscriber subOdomAftMapped;
    ros::Subscriber subSegmentationRadius;
    ros::Subscriber subPeakDistanceSum;

public:
    keyPointSave() : nh("~")
    {
        subPeakNum = nh.subscribe<std_msgs::UInt8>("/intersection/peakNum", 1, &keyPointSave::peakNumHandler, this);
        subClusterNum = nh.subscribe<std_msgs::UInt8>("/intersection/clusterNum", 1, &keyPointSave::clusterNumHandler, this);
        subSegmentationRadius = nh.subscribe<std_msgs::UInt8>("/intersection/segmentationRadius", 1, &keyPointSave::segmentationRadiusHandler, this);
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 1, &keyPointSave::odomAftMapped, this);
        subIntersectionVerified = nh.subscribe<std_msgs::Bool>("/intersection/intersectionVerified", 1, &keyPointSave::intersectionVerifiedHandler, this);
        subPeakDistanceSum = nh.subscribe<std_msgs::Float32>("/intersection/peakDistance_Max", 1, &keyPointSave::peakDistanceSumHandler, this);
        allocateMemory();
    }

    void allocateMemory()
    {
        ROS_DEBUG("allocateMemory");
    }

    void peakNumHandler(std_msgs::UInt8 msg)
    {
        peakNum = msg.data;
        ROS_DEBUG_STREAM("Peak Number  =    " << peakNum);
    }
    void intersectionVerifiedHandler(std_msgs::Bool msg)
    {
        intersectionVerified = msg.data;
    }

    void clusterNumHandler(std_msgs::UInt8 msg)
    {
        clusterNum = msg.data;
        ROS_DEBUG_STREAM("Cluster Number  =    " << clusterNum);
    }

    void segmentationRadiusHandler(std_msgs::UInt8 msg)
    {
        segmentationRadius = msg.data;
        ROS_DEBUG_STREAM("Cluster Number  =    " << clusterNum);
    }

    void peakDistanceSumHandler(std_msgs::Float32 msg)
    {
        peakDistance_Max = msg.data;
        ROS_DEBUG_STREAM("Peak Distance Summary  =    " << peakDistance_Max);
    }

    void odomAftMapped(nav_msgs::Odometry msg)
    {
        currentPosition.x = msg.pose.pose.position.x;
        currentPosition.y = msg.pose.pose.position.y;
        currentPosition.z = msg.pose.pose.position.z;
        ROS_DEBUG_STREAM("This Key Point: x = " << currentPosition.x << " y = " << currentPosition.y << " z = " << currentPosition.z);
    }

    void judge()
    {
        //初次赋值
        if (peakNum_Pre == -1)
            peakNum_Pre = peakNum;
        if (clusterNum_Pre == -1)
            clusterNum_Pre = clusterNum;

        //判断变化
        if (!record_Bool && intersectionVerified && peakNum > 2)
        {
            if (clusterNum_Pre <= 2 && clusterNum > 2)
            {
                record_Bool = true;
                begin = ros::Time::now();
                clusterNum_max = 0;
                peakNum_max = 0;
                peakDistance_min = 20;
                thisNum_Vec.clear();
                currentPosition_Vec.clear();
                counter += 1;

                ROS_INFO("Start Recording Key Points.");
            }
        }
        else if (record_Bool)
        {
            if (clusterNum_Pre > 2 && clusterNum <= 2 || !intersectionVerified)
            {
                record_Bool = false;
                ros::Duration duration = ros::Time::now() - begin;
                interval = duration.toSec();

                if (interval < 1)
                {
                    ROS_INFO("Cancel Recording Key Points.");
                    counter -= 1;
                }
                else
                {
                    keyPoints_Vec.push_back(std::pair<short int, pcl::PointXYZ>(counter, thisKeyPoint));
                    radius_Vec.push_back(segmentationRadius_max);

                    numOfIntersection_Vec.push_back(std::pair<short int, short int>(clusterNum_max, peakNum_max));

                    ROS_INFO_STREAM("\033[1;32m---->\033[0m End Recording Key Points:  " << counter);
                }
            }
        }
        ROS_DEBUG("judge");
    }

    void recordKeyPoint()
    {
        if (!currentPosition_Vec.empty() && getDistanceOf2Point(currentPosition, currentPosition_Vec.back().first) < 0.005)
            return;

        if (segmentationRadius > segmentationRadius_Max)
            return;

        currentPosition_Vec.push_back(std::pair<pcl::PointXYZ, short int>(currentPosition, segmentationRadius));
        thisNum_Vec.push_back(std::pair<short int, short int>(clusterNum, peakNum));

        if (clusterNum_max <= clusterNum)
        {
            clusterNum_max = clusterNum;
            if (peakDistance_min > peakDistance_Max)
            {
                peakDistance_min = peakDistance_Max;
                thisKeyPoint = currentPosition;
                segmentationRadius_max = segmentationRadius;
                peakNum_max = peakNum;
            }
        }

        ROS_DEBUG("recordKeyPoint");
    }

    float getDistanceOf2Point(pcl::PointXYZ point1, pcl::PointXYZ point2)
    {
        float distance = sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2));
        return distance;
    }

    void update()
    {
        peakNum_Pre = clusterNum;
        clusterNum_Pre = clusterNum;

        ROS_DEBUG("update");
    }

    void pointsSave()
    {
        ROS_INFO("============== pointsSave ==============");
        std::ofstream outfile;

        outfile.open("/home/lsj/dev/Mine_WS/data/" + save_Name + ".txt");

        outfile << std::setiosflags(std::ios::left) << std::setw(10) << "index" << std::setw(15) << "keyPoint.x" << std::setw(15) << "keyPoint.y"
                << std::setw(15) << "keyPoint.z" << std::setw(20) << "cluster_number" << std::setw(20) << "peak_number" << std::setw(10) << "radius" << std::endl;

        std::vector<std::pair<short int, short int>>::iterator iterNum = numOfIntersection_Vec.begin();
        std::vector<short int>::iterator iterRadius = radius_Vec.begin();
        for (std::vector<std::pair<short int, pcl::PointXYZ>>::iterator iter = keyPoints_Vec.begin(); iter != keyPoints_Vec.end(); ++iter)
        {
            ROS_INFO("====================================================");
            ROS_INFO_STREAM("index == " << iter->first << std::endl);
            ROS_INFO_STREAM("keyPointPosition == " << iter->second << std::endl);
            ROS_INFO_STREAM("peak_number == " << iterNum->first << std::endl);
            ROS_INFO_STREAM("cluster_number == " << iterNum->second << std::endl);

            outfile << std::setiosflags(std::ios::left) << std::setw(10) << iter->first << std::setprecision(3) << std::setw(15) << iter->second.x << std::setw(15) << iter->second.y
                    << std::setw(15) << iter->second.z << std::setw(20) << iterNum->first << std::setw(20) << iterNum->second << std::setw(10) << *iterRadius << std::endl;

            ++iterNum;
        }

        ROS_INFO("====================================================");
        ROS_INFO_STREAM("Size of numOfIntersection_Vec == " << numOfIntersection_Vec.size());
        ROS_INFO_STREAM("Size of keyPoints_Vec == " << keyPoints_Vec.size());

        outfile.close();
        ROS_INFO("============== pointsSave ==============");
    }

    void run()
    {

        getDynamicParameter();

        judge();

        if (record_Bool)
            recordKeyPoint();

        if (save_Bool)
        {
            pointsSave();
            ros::param::set("/intersection/bool_save", false);
        }

        update();
    }

    void getDynamicParameter()
    {
        ros::param::get("/intersection/save_name", save_Name);
        ros::param::get("/intersection/bool_save", save_Bool);

        ROS_DEBUG("getDynamicParameter");
    }
};

//动态调参
void callback(perception::param_Config &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %s %s",
             config.save_name.c_str(),
             config.bool_save ? "True" : "False");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyPointSave");

    /*动态参数调节*/
    dynamic_reconfigure::Server<perception::param_Config> server;
    dynamic_reconfigure::Server<perception::param_Config>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    /*动态参数调节*/

    ROS_INFO("\033[1;32m---->\033[0m Key Point Save Started.");

    keyPointSave KP;

    ros::Rate rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        KP.run();
        rate.sleep();
    }

    return 0;
}