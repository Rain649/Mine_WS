#include<ros/ros.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>

#include "std_msgs/Int16.h"

class keyPointSave
{
    private:
        uint16_t clusterNum;
        ros::NodeHandle nh;
        ros::Subscriber subClusterNum;
    
public:
    keyPointSave() : nh("~")
    {  
        subClusterNum = nh.subscribe<std_msgs::Int16>("/intersection/clusterNum", 1, &keyPointSave::clusterNumHandler, this);

    }

    void clusterNumHandler(std_msgs::Int16 msg)
    {
        clusterNum = msg.data;
        ROS_INFO_STREAM("Cluster Number  =    " << clusterNum);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc,argv,"keyPointSave");
    
    ROS_INFO("\033[1;32m---->\033[0m Key Point Save Started.");

    keyPointSave KP;

    ros::Rate rate(5);
    while (ros::ok())
    {
        ros::spinOnce();
        // KP.run();
        rate.sleep();
    }

    return 0;
}