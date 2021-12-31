#ifndef DIJ_H_
#define DIJ_H_

#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#include <queue>
#include <unordered_set>
#include <unordered_map>

#include "topoMap.h"

namespace dij
{

class Vertex
{
public:
    Vertex() {}

    Vertex(int id_) : id(id_)
    {
        cost = DBL_MAX;

        parent_id = -1;
    }

    int id;

    double cost;

    int parent_id;
};

struct VertexComparison
{
    bool operator() (const Vertex& a, const Vertex& b)
    {
        return a.cost > b.cost;
    }
};

class Dij
{
public:
    Dij(ros::NodeHandle& nh);

    bool FindPath(TopoMap& topoMap);

    void GenPath(void);

    void PubPath(void);

    void PubTarget(TopoMap& topoMap);

    int RATE;

    int source_id;

    int target_id;

private:
    ros::Publisher PathPuber;

    std_msgs::Int32MultiArray path_msg;

    ros::Subscriber ForkSuber;

    void ForkCallback(const std_msgs::Bool& msg);

    bool fork_flag, fork_flag_old;

    ros::Publisher TargetPuber;

    nav_msgs::Odometry target_msg;

    std::vector<int> path;

    int vertex_index;

    void ClearTargetMsg();

    bool update_flag;

    std::priority_queue<Vertex, std::vector<Vertex>, VertexComparison> frontier;

    std::unordered_map<int, Vertex> reached;
};

} // namespace dij

#endif  // DIJ_H_