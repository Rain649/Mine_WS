#include "ros/ros.h"
#include "perception/mineServer.h"

int main(int argc, char **argv)
{
    int nodeID_Index{-1};
    std::vector<int> pathPlanned;

    ros::init(argc, argv, "nodeRequest");

    // 从终端命令行获取 mode, nodeID_index
    if (argc != 3)
    {
        ROS_INFO("usage: input 2 parameters: mode nodeID_index");
        return 1;
    }

    ros::NodeHandle nh;

    // 创建一个client，请求service
    ros::ServiceClient client = nh.serviceClient<perception::mineServer>("/service/node_ID_Manager");

    // 创建service消息
    perception::mineServer addsrv;
    addsrv.request.mode = atol(argv[1]);
    addsrv.request.nodeID_Index = atol(argv[2]);

    // 发布service请求，等待应答结果
    if (client.call(addsrv))
    {
        switch (addsrv.request.mode)
        {
        case 1:
            pathPlanned = addsrv.response.pathPlanned;
            printf("Path: ");
            for (auto i : pathPlanned)
                printf("%d -> ", i);
            printf("\n");
            break;

        case 2:
            nodeID_Index = addsrv.response.nodeID_Index;
            ROS_INFO("nodeID_index: %d", nodeID_Index);
            break;
        case 3:
            nodeID_Index = addsrv.response.nodeID_Index;
            ROS_INFO("nodeID_index: %d", nodeID_Index);
            break;

        default:
            break;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service node_ID_Manager");
    }


    ros::shutdown();
    return 0;
}