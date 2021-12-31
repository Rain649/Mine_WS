#ifndef LKA_PLANNER_H_
#define LKA_PLANNER_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>

class LKA_planner
{
private:
    ros::Subscriber CoefficientSub;

    void CoefficientCallback(const std_msgs::Float32MultiArray& msg);

    ros::Subscriber RangeSub;

    void RangeCallback(const std_msgs::Float32MultiArray& msg);

    std::vector<float> coeff_path;

    float X_min, X_max;

    float X_step;

    float Y_bias;

    // ros::Publisher PathPub;

    geometry_msgs::PoseStamped node_msg;

public:
    LKA_planner(ros::NodeHandle& nh);

    bool CallbackFlag(void);

    void GenLKAPath(void);

    bool coeff_flag;

    bool range_flag;

    nav_msgs::Path path_msg;

    int coeff_counter, range_counter;

    int Counter_MAX;
};

#endif // LKA_PLANNER_H_