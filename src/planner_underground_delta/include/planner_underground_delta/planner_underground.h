#ifndef PLANNER_UNDERGROUND_H
#define PLANNER_UNDERGROUND_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>

#include "dubins_bug_delta/dubins_bug.h"
#include "lka_planner/lka_planner.h"

class UG_planner: public LKA_planner, public DBPlanner
{
private:
    ros::Subscriber ForkSub;

    void ForkCallback(const std_msgs::Bool& msg);

    void ResetMsgFlag(void);

    int valid_counter;

    int VALIDMAX;

    ros::Publisher PathPub;

    nav_msgs::Path path_msg;

public:

    UG_planner(ros::NodeHandle& nh): LKA_planner(nh), DBPlanner(nh)
    {
        RATE = 20;

        ForkSub = nh.subscribe("/intersectionDetection/intersectionVerified", 10, &UG_planner::ForkCallback, this);

        fork_flag = false;

        fork_flag_old = false;

        VALIDMAX = 2 * RATE;

        PathPub = nh.advertise<nav_msgs::Path>("/path_ug_vehicle", 10);
    }

    int RATE;

    bool fork_flag, fork_flag_old;

    bool DBMsgIsOk(void);

    bool LKAMsgIsOk(void);

    bool UGMsgIsOk(void);

    void PubLKAPathMsg(void);

    void PubDBPathMsg(void);
};

#endif // PLANNER_UNDERGROUND_H