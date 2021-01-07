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
    short int segmentationRadius = 0;
    short int clusterNum_Pre = -1;
    short int peakNum_Pre = -1;
    short int counter = 0;
    short int clusterNum_max = 0;
    short int peakNum_max = 0;

    double interval;

    bool record_Bool = false;
    bool save_Bool = false;

    std::string save_Name;
    std::vector<short int> radius_Vec;
    std::vector<std::pair<short int, short int>> thisNum_Vec;
    std::vector<std::pair<pcl::PointXYZ, short int>> thisKeyPoint_Vec;
    std::vector<std::pair<short int, pcl::PointXYZ>> keyPoints_Vec;
    std::vector<std::pair<short int, short int>> numOfIntersection_Vec;

    pcl::PointXYZ thisKeyPoint;

    ros::NodeHandle nh;
    ros::Time begin;
    ros::Subscriber subPeakNum;
    ros::Subscriber subClusterNum;
    ros::Subscriber subOdomAftMapped;
    ros::Subscriber subSegmentationRadius;
public:
    keyPointSave() : nh("~")
    {
        subPeakNum = nh.subscribe<std_msgs::UInt8>("/intersection/peakNum", 1, &keyPointSave::peakNumHandler, this);
        subClusterNum = nh.subscribe<std_msgs::UInt8>("/intersection/clusterNum", 1, &keyPointSave::clusterNumHandler, this);
        subSegmentationRadius = nh.subscribe<std_msgs::UInt8>("/intersection/segmentationRadius", 1, &keyPointSave::segmentationRadiusHandler, this);
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

    void segmentationRadiusHandler(std_msgs::UInt8 msg)
    {
        segmentationRadius = msg.data;

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
        if (peakNum_Pre == -1)
            peakNum_Pre = peakNum;
        if (clusterNum_Pre == -1)
            clusterNum_Pre = clusterNum;

        //判断变化
        if (clusterNum_Pre <= 2 && clusterNum > 2)
        {
            record_Bool = true;
            begin = ros::Time::now();
            clusterNum_max = 0;
            peakNum_max = 0;
            thisNum_Vec.clear();
            thisKeyPoint_Vec.clear();
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
                ROS_INFO("Cancel Recording Key Points.");
                counter -= 1;
            }
            else
            {
                //保存预测数值和聚类数值
                // std::vector<int>::iterator index_NumMax = find(thisNum_Vec.begin(), thisNum_Vec.end(), clusterNum_max); //查找target
                // std::vector<std::pair<short int，short int>>::iterator iterNum = thisNum_Vec.begin();
                // std::vector<short int> Max_Vec;
                // std::vector<std::pair<pcl::PointXYZ, short int>>::iterator iterKeyPoint = thisKeyPoint_Vec.begin();
                // while ((iterNum = std::find_if(iterNum, thisNum_Vec.end(), (clusterNum_max, peakNum_max))) != thisNum_Vec.end())  //当iter等于A.end()时，意味着处理完毕，退出循环
                // {
                //     // indexOfMax_Vec.push_back()
                //     push_back
                    
                //     iterKeyPoint++;
                //     iterNum++;  //iter加1，作为下次查找的起点。
                // }

                std::vector<std::pair<short int,short int>>::iterator iterNum = thisNum_Vec.begin();
                std::vector<std::pair<pcl::PointXYZ,short int>>::iterator iterKeyPoint = thisKeyPoint_Vec.begin();
                while(iterNum!= thisNum_Vec.end())
                {
                    if(iterNum->first!=clusterNum_max||iterNum->second!=peakNum_max)
                    {
                        iterNum = thisNum_Vec.erase(iterNum);
                        iterKeyPoint = thisKeyPoint_Vec.erase(iterKeyPoint);
                    }
                    else
                    {
                        iterNum++;
                        iterKeyPoint++;
                    }
                }

                if(thisKeyPoint_Vec.size() == 0)
                {
                    ROS_ERROR("thisKeyPoint_Vec has no data!!!!!!!!!");
                    ROS_ERROR("Cancel Recording Key Points.");
                    counter -= 1;

                    return;
                }

                keyPoints_Vec.push_back(std::pair<short int, pcl::PointXYZ>(counter, thisKeyPoint_Vec[(thisKeyPoint_Vec.size()+1)/2].first));
                radius_Vec.push_back(thisKeyPoint_Vec[(thisKeyPoint_Vec.size()+1)/2].second);
                numOfIntersection_Vec.push_back(std::pair<short int, short int>(clusterNum_max , peakNum_max));

                ROS_INFO_STREAM("\033[1;32m---->\033[0m End Recording Key Points:  " << counter);
            }
        }
        ROS_DEBUG("judge");
    }

    void recordKeyPoint()
    {
        if (!thisKeyPoint_Vec.empty())
        {
            if (getDistanceOf2Point(thisKeyPoint, thisKeyPoint_Vec.back().first)<0.005) return;
        }
        
        if(segmentationRadius>16)
        {
            return;
        }

        thisKeyPoint_Vec.push_back(std::pair<pcl::PointXYZ, short int>(thisKeyPoint, segmentationRadius));
        thisNum_Vec.push_back(std::pair<short int,short int>(clusterNum,peakNum));

        if (peakNum_max < peakNum)
        {
            peakNum_max = peakNum;
            if (clusterNum_max < clusterNum)
                clusterNum_max = clusterNum;
        }

        ROS_DEBUG("recordKeyPoint");
    }

    float getDistanceOf2Point(pcl::PointXYZ point1,pcl::PointXYZ point2)
    {
        float distance = sqrt(pow(point1.x - point2.x,2) + pow(point1.y - point2.y,2) + pow(point1.z - point2.z,2));
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
            ROS_INFO_STREAM("keyPointPosition == " << iter-> second << std::endl);
            ROS_INFO_STREAM("peak_number == " << iterNum->first << std::endl);
            ROS_INFO_STREAM("cluster_number == " << iterNum-> second << std::endl);

            outfile << std::setiosflags(std::ios::left) << std::setw(10) << iter->first<< std::setprecision(3) << std::setw(15) << iter->second.x << std::setw(15) << iter->second.y 
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