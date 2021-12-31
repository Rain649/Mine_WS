#include "planner_underground_delta/planner_underground.h"

void UG_planner::ForkCallback(const std_msgs::Bool& msg)
{
    fork_flag_old = fork_flag;

    fork_flag = msg.data;

    if (fork_flag && !fork_flag_old)
    {
        ResetMsgFlag();
    }

    valid_counter = 0;
}

void UG_planner::ResetMsgFlag(void)
{
    coeff_flag = false;

    range_flag = false;

    odom_flag = false;

    target_flag = false;

    lidar_flag = false;
}

bool UG_planner::DBMsgIsOk(void)
{
    std::cout << "DB_planner state:" << std::endl;

    return DBPlanner::CallbackFlag();
}

bool UG_planner::LKAMsgIsOk(void)
{
    std::cout << "LKA_planner state:" << std::endl;

    return LKA_planner::CallbackFlag();
}

bool UG_planner::UGMsgIsOk(void)
{
    ++valid_counter;

    if (valid_counter < VALIDMAX)
    {
        return true;
    }
    else
    {
        std::cout << "Msg about fork can not receive" << std::endl;

        valid_counter = VALIDMAX;

        return false;
    }
}

void UG_planner::PubLKAPathMsg(void)
{
    path_msg = LKA_planner::path_msg;

    PathPub.publish(path_msg);
}

void UG_planner::PubDBPathMsg(void)
{
    DBPlanner::PubPathMsg();

    path_msg = DBPlanner::path_pdt_veh_msg;

    PathPub.publish(path_msg);
}
