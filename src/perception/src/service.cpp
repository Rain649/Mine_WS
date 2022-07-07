#include "ros/ros.h"
#include "perception/mineServer.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <yaml-cpp/yaml.h>

class Service
{
private:
    static std::vector<int> pathPlanned;

    static int intersectionID_id;

    ros::NodeHandle nh;

    ros::Subscriber path_sub;
    ros::Subscriber nodeIDIndex_sub;

    ros::ServiceServer service;

public:
    Service() : nh("~")
    {
        //加载参数
        std::string path_dir;
        nh.param<std::string>("path_dir", path_dir, "src/common/map/PathPlanned.yaml");
        // nh.param<std::string>("path_dir", path_dir, "/home/lsj/dev/IntersectionPerception_Planning_Control/src/common/map/PathPlanned.yaml");

        ROS_INFO("simu_data path: %s", path_dir.c_str());

        nodeIDIndex_sub = nh.subscribe<std_msgs::Int32ConstPtr>("/navigation/intersectionID_id", 1, &Service::nodeIDHandler, this);

        service = nh.advertiseService("node_ID_Manager", IDCallback);

        YAML::Node config = YAML::LoadFile(path_dir);
        YAML::Node pathPlanned_YN = config["pathPlanned"];
        for (const auto i : pathPlanned_YN)
        {
            pathPlanned.push_back(i.as<int>());
        }

        printf("Path: ");
        for (auto i : pathPlanned)
            printf("%d -> ", i);
        printf("\n");

        ros::spin();
    }
    void pathHandler(const std_msgs::Int32MultiArray msg)
    {
        pathPlanned.clear();
        std::cout << "< Path";
        for (size_t i = 0; i < msg.data.size(); ++i)
        {
            std::cout << " > " << msg.data[i];
            pathPlanned.push_back(msg.data[i]);
        }
        std::cout << std::endl;
        path_sub.shutdown();
    }
    void nodeIDHandler(const std_msgs::Int32ConstPtr msg)
    {
        intersectionID_id = msg->data;
    }

    static bool IDCallback(perception::mineServer::Request &req, perception::mineServer::Response &res)
    {
        switch (req.mode)
        {
        case 1:
            res.pathPlanned = pathPlanned;
            break;
        case 2:
            res.nodeID_Index = intersectionID_id;
            break;
        case 3:
            intersectionID_id = req.nodeID_Index;
            res.nodeID_Index = req.nodeID_Index;
            break;

        default:
            break;
        }

        ROS_INFO("request: mode=%ld\n", (long int)req.mode);
        // ROS_INFO("request: mode=%ld, nodeID_Index=%ld", (long int)req.mode, (long int)req.nodeID_Index);
        // ROS_INFO("sending back response: [%ld]", (long int)res.nodeID_Index);
        return true;
    }
};

int Service::intersectionID_id = 0;
std::vector<int> Service::pathPlanned = {};

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "service");

    Service service_obj;

    return 0;
}