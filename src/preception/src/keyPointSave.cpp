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
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include <vector>
#include <string>

#include <dynamic_reconfigure/server.h>
#include <preception/param_Config.h>

class keyPointSave
{
private:
    short int clusterNum = 0;
    short int peakNum = 0;
    short int clusterNum_Pre = 255;
    short int peakNum_Pre = 255;
    short int counter = 0;
    short int clusterNum_max = 0;
    short int peakNum_max = 0;

    double interval;

    bool record_Bool = false;
    bool save_Bool = false;

    std::string save_Name;

    std::vector<std::pair<short int, pcl::PointXYZ>> keyPoints;
    std::vector<std::pair<short int, short int>> numOfIntersection;

    pcl::PointXYZ thisKeyPoint;

    ros::NodeHandle nh;
    ros::Time begin;
    ros::Subscriber subPeakNum;
    ros::Subscriber subClusterNum;
    ros::Subscriber subOdomAftMapped;

public:
    keyPointSave() : nh("~")
    {
        subPeakNum = nh.subscribe<std_msgs::UInt8>("/intersection/peakNum", 1, &keyPointSave::peakNumHandler, this);
        subClusterNum = nh.subscribe<std_msgs::UInt8>("/intersection/clusterNum", 1, &keyPointSave::clusterNumHandler, this);
        subOdomAftMapped = nh.subscribe<nav_msgs::Odometry>("/aft_mapped_to_init", 1, &keyPointSave::odomAftMapped, this);

        allocateMemory();
    }

    void allocateMemory()
    {
        ROS_DEBUG("allocateMemory");
    }

    void peakNumHandler(std_msgs::UInt8 msg)
    {
        peakNum = msg.data;

        // ROS_INFO_STREAM("Peak Number  =    " << peakNum);
    }

    void clusterNumHandler(std_msgs::UInt8 msg)
    {
        clusterNum = msg.data;

        // ROS_INFO_STREAM("Cluster Number  =    " << clusterNum);
    }

    void odomAftMapped(nav_msgs::Odometry msg)
    {
        thisKeyPoint.x = msg.pose.pose.position.x;
        thisKeyPoint.y = msg.pose.pose.position.y;
        thisKeyPoint.z = msg.pose.pose.position.z;

        // ROS_INFO_STREAM("This Key Point: x = " << thisKeyPoint.x << " y = " << thisKeyPoint.y << " z = " << thisKeyPoint.z);
    }

    void judge()
    {
        //初次赋值
        if (peakNum_Pre == 255)
            peakNum_Pre = peakNum;
        if (clusterNum_Pre == 255)
            clusterNum_Pre = clusterNum;

        //判断变化
        if (clusterNum_Pre <= 2 && clusterNum > 2)
        {
            record_Bool = true;
            begin = ros::Time::now();
            clusterNum_max = 0;
            peakNum_max = 0;
            counter += 1;

            ROS_INFO("Start Recording Key Points.");
        }

        if (clusterNum_Pre > 2 && clusterNum <= 2)
        {
            record_Bool = false;
            ros::Duration duration = ros::Time::now() - begin;
            interval = duration.toSec();

            if (interval < 2)
            {
                //持续时间小，视为杂波，剔除位置信息
                while (keyPoints.back().first == counter)
                {
                    keyPoints.pop_back();
                }
                counter -= 1;

                ROS_INFO("Cancel Recording Key Points.");
            }
            else
            {
                //保存预测数值和聚类数值
                numOfIntersection.push_back(std::pair<short int, short int>(peakNum_max, clusterNum_max));

                ROS_INFO("End Recording Key Points.");
            }
        }
        ROS_DEBUG("judge");
    }

    void recordKeyPoint()
    {
        keyPoints.push_back(std::pair<short int, pcl::PointXYZ>(counter, thisKeyPoint));
        if (clusterNum_max < clusterNum)
            clusterNum_max = clusterNum;
        if (peakNum_max < peakNum)
            peakNum_max = peakNum;

        ROS_DEBUG("recordKeyPoint");
    }

    void update()
    {
        peakNum_Pre = clusterNum;
        clusterNum_Pre = clusterNum;

        ROS_DEBUG("update");
    }

    void pointSave()
    {
        ROS_INFO("============== pointSave ==============");
        std::ofstream outfile;

        outfile.open("/home/lsj/dev/Mine_WS/data/" + save_Name+".txt");

        outfile << "peak_number"<< "    ";
        outfile << "cluster_number" << std::endl;

        for (std::vector<std::pair<short int, short int>>::iterator iter = numOfIntersection.begin(); iter != numOfIntersection.end(); ++iter)
        {
            ROS_INFO_STREAM("first  =    " << iter->first);
            outfile << iter->first << "    ";
            ROS_INFO_STREAM("second  =    " << iter->second);
            outfile << iter->second << std::endl;
        }

        outfile.close();

        ROS_INFO("============== pointSave ==============");
    }

    void run()
    {
        getDynamicParameter();

        judge();

        if (record_Bool)
            recordKeyPoint();

        if (save_Bool)
        {
            pointSave();
            ros::param::set("/intersection/bool_save",false);
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
void callback(preception::param_Config &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %s %s",
             config.save_name.c_str(),
             config.bool_save ? "True" : "False");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyPointSave");

    //动态参数调节
    dynamic_reconfigure::Server<preception::param_Config> server;
    dynamic_reconfigure::Server<preception::param_Config>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ROS_INFO("\033[1;32m---->\033[0m Key Point Save Started.");

    keyPointSave KP;

    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        KP.run();
        rate.sleep();
    }

    return 0;
}