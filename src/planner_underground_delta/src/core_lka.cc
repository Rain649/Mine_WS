#include "lka_planner/lka_planner.h"

LKA_planner::LKA_planner(ros::NodeHandle& nh)
{
    std::string topic_rightCoefficient;
    nh.getParam("topic_rightCoefficient", topic_rightCoefficient);
    CoefficientSub = nh.subscribe(topic_rightCoefficient, 10, &LKA_planner::CoefficientCallback, this);

    std::string topic_rightRange;
    nh.getParam("topic_rightRange", topic_rightRange);
    RangeSub = nh.subscribe(topic_rightRange, 10, &LKA_planner::RangeCallback, this);
    
    // PathPub = nh.advertise<nav_msgs::Path>("/path_lka", 10);

    // path_msg.header.frame_id = "map";
    nh.getParam("frame_id", path_msg.header.frame_id);

    coeff_flag = false;

    range_flag = false;

    // X_min = -0.5;

    // X_max = 10;

    // X_step = 0.2;
    nh.getParam("X_step", X_step);

    // Y_bias = 2.5;
    nh.getParam("Y_bias", Y_bias);

    coeff_counter = 0;

    range_counter = 0;

    Counter_MAX = 1 * 20;
}

void LKA_planner::RangeCallback(const std_msgs::Float32MultiArray& msg)
{
    X_max = msg.data[0];

    X_min = msg.data[1];

    range_flag = true;

    range_counter = 0;
}

void LKA_planner::CoefficientCallback(const std_msgs::Float32MultiArray& msg)
{
    coeff_path.resize(msg.data.size());

    for (size_t i = 0; i < coeff_path.size(); i++)
    {
        coeff_path[i] = msg.data[i];
    }

    coeff_flag = true;

    coeff_counter = 0;
}

bool LKA_planner::CallbackFlag(void)
{
    if (coeff_counter > Counter_MAX)
    {
        coeff_flag = false;

        coeff_counter = Counter_MAX;
    }

    if (range_counter > Counter_MAX)
    {
        range_flag = false;

        range_counter = Counter_MAX;
    }
    
    std::cout << "coeff_flag = " << coeff_flag << ", " 
              << "range_flag " << range_flag << std::endl;

    return coeff_flag && range_flag;
}

void LKA_planner::GenLKAPath(void)
{
    path_msg.poses.clear();

    for (float i = X_min; i <= X_max; i += X_step)
    {
        node_msg.pose.position.x = i;

        node_msg.pose.position.y = 0;

        for (size_t j = 0; j < coeff_path.size(); j++)
        {
            node_msg.pose.position.y += coeff_path[j] * pow(i, j);
        }

        node_msg.pose.position.y += Y_bias;

        path_msg.poses.push_back(node_msg);
    }

    path_msg.header.stamp = ros::Time::now();

    // coeff_flag = false;

    // range_flag = false;

    ++coeff_counter;
    ++range_counter;
}
