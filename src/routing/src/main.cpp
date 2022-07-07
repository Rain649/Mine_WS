// #include "routing_delta/dij.h"

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <yaml-cpp/yaml.h>


// #include "topoMap.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "routing");

    ros::NodeHandle nh("~");

    ros::Publisher pathPlanned_pub = nh.advertise<std_msgs::Int32MultiArray>("/pathplanned", 1);

    std::string fin = "src/common/map/PathPlanned.yaml";
    YAML::Node config = YAML::LoadFile(fin);
    YAML::Node pathPlanned_YN = config["pathPlanned"];
    std_msgs::Int32MultiArray pathPlanned;
    for (const auto i : pathPlanned_YN)
    {
        pathPlanned.data.push_back(i.as<int>());
        std::cout << i.as<int>() << std::endl;
    }

    ros::Rate rate(10);

    while (ros::ok())
    {
        ros::spinOnce();

        pathPlanned_pub.publish(pathPlanned);

        rate.sleep();
    }

    return 0;
}
