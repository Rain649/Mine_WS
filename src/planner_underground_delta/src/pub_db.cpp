#include "dubins_bug_delta/dubins_bug.h"

void DBPlanner::PubPathMsg(void)
{
    // path_local
    path_local_msg.poses.resize(path_local.size());

    for (size_t i = 0; i < path_local.size(); i++)
    {
        path_local_msg.poses[i].pose.position.x = path_local[i].x;
        path_local_msg.poses[i].pose.position.y = path_local[i].y;
        path_local_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(path_local[i].phi);
    }
    
    path_local_msg.header.stamp = ros::Time::now();

    PathLocalPub.publish(path_local_msg);

    // path_local_veh
    path_local_veh_msg.poses.resize(path_local_veh.size());

    for (size_t i = 0; i < path_local_veh.size(); i++)
    {
        path_local_veh_msg.poses[i].pose.position.x = path_local_veh[i].x;
        path_local_veh_msg.poses[i].pose.position.y = path_local_veh[i].y;
        path_local_veh_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(path_local_veh[i].phi);
    }
    
    path_local_veh_msg.header.stamp = ros::Time::now();

    PathLocalVehPub.publish(path_local_veh_msg);

    // path_pdt
    path_pdt_msg.poses.resize(path_pdt.size());

    for (size_t i = 0; i < path_pdt.size(); i++)
    {
        path_pdt_msg.poses[i].pose.position.x = path_pdt[i].x;
        path_pdt_msg.poses[i].pose.position.y = path_pdt[i].y;
        path_pdt_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(path_pdt[i].phi);
    }
    
    path_pdt_msg.header.stamp = ros::Time::now();

    PathPdtPub.publish(path_pdt_msg);

    // path_pdt_veh
    path_pdt_veh_msg.poses.resize(path_pdt_veh.size());

    for (size_t i = 0; i < path_pdt_veh.size(); i++)
    {
        path_pdt_veh_msg.poses[i].pose.position.x = path_pdt_veh[i].x;
        path_pdt_veh_msg.poses[i].pose.position.y = path_pdt_veh[i].y;
        path_pdt_veh_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(path_pdt_veh[i].phi);
    }
    
    path_pdt_veh_msg.header.stamp = ros::Time::now();

    PathPdtVehPub.publish(path_pdt_veh_msg);

    // test
    // target in vehicle frame
    target_veh_msg.pose.pose.position.x = target_x_veh;
    target_veh_msg.pose.pose.position.y = target_y_veh;

    double target_yaw_veh = final_state[2] - current_state.yaw;

    while (target_yaw_veh > M_PI)
    {
        target_yaw_veh -= 2*M_PI;
    }
    while (target_yaw_veh <= -M_PI)
    {
        target_yaw_veh += 2*M_PI;
    }
    
    target_veh_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw_veh);

    target_veh_msg.header.stamp = ros::Time::now();

    TargetInVehPub.publish(target_veh_msg);
    // test
}