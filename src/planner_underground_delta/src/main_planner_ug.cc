#include "planner_underground_delta/planner_underground.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "planner_ug");

    ros::NodeHandle nh;

    UG_planner planner_ug(nh);

    DubinsStateSpace db(planner_ug.steer_radius);

    ros::Rate loop_rate(planner_ug.RATE);

    while (ros::ok())
    {
        ros::spinOnce();

        if (!planner_ug.UGMsgIsOk())
        {
            std::cout << std::endl;

            loop_rate.sleep();

            continue;
        }
        
        if (planner_ug.fork_flag)
        {
            if (planner_ug.DBMsgIsOk())
            {
                planner_ug.BugPlanner(db);
                
                if (planner_ug.UpdatePdtPath())
                {
                    planner_ug.GenPdtPath();
                }

                planner_ug.PubDBPathMsg();
            }
        }
        else
        {
            if (planner_ug.LKAMsgIsOk())
            {
                planner_ug.GenLKAPath();

                planner_ug.PubLKAPathMsg();
            }
        }

        std::cout << std::endl;

        loop_rate.sleep();
    }
       
    return 0;
}

