#include "control_underground_delta/path_follow.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_follow");
    
    ros::NodeHandle nh("~");

    PathFollow path_follow(nh);

    ros::Rate loop_rate(path_follow.RATE);

    while (ros::ok())
    {
        ros::spinOnce();

        if (!path_follow.CallbackFlag())
        {
            path_follow.Stop();

            path_follow.PubCmd();

            loop_rate.sleep();

            continue;
        }

        path_follow.PushPathNode();

        path_follow.GenLatCmd();

        path_follow.GenLonCmd();

        path_follow.PubCmd();

        std::cout << std::endl;

        loop_rate.sleep();
    }
    
    return 0;
}