#include "routing_delta/dij.h"
#include "topoMap.h"

namespace dij
{

Dij::Dij(ros::NodeHandle& nh)
{
    std::string topic_routing_path;
    nh.getParam("topic_routing_path", topic_routing_path);
    PathPuber = nh.advertise<std_msgs::Int32MultiArray>(topic_routing_path, 1);

    std::string topic_intersection;
    nh.getParam("topic_intersection", topic_intersection);
    ForkSuber = nh.subscribe(topic_intersection, 1, &Dij::ForkCallback, this);

    RATE = 20;

    fork_flag = false;

    fork_flag_old = false;

    std::string topic_target_fork;
    nh.getParam("topic_target_fork", topic_target_fork);
    // TargetPuber = nh.advertise<nav_msgs::Odometry>(topic_target_fork, 1);
    TargetPuber = nh.advertise<nav_msgs::Path>(topic_target_fork, 1);

    nh.getParam("frame_id", target_msg.header.frame_id);

    update_flag = false;
}

void Dij::ForkCallback(const std_msgs::Bool& msg)
{    
    fork_flag_old = fork_flag;

    fork_flag = msg.data;

    if (fork_flag && !fork_flag_old)
    {
        update_flag = true;
    }
}

bool Dij::FindPath(TopoMap& topoMap)
{
    Vertex source(source_id);

    source.cost = 0;
    
    reached.emplace(source_id, source);

    frontier.push(source);

    Vertex vertex_cur;

    std::vector<int> children_id;
    
    while (!frontier.empty())
    {
        vertex_cur = frontier.top();

        if (vertex_cur.id == target_id)
        {
            return true;
        }

        frontier.pop();

        children_id = topoMap.get_linkedVertex_Vec(vertex_cur.id);

        for (auto &&id : children_id)
        {            
            double cost_tmp = reached[vertex_cur.id].cost + 
                              topoMap.get_cost(vertex_cur.id, id);
            
            if (!reached.count(id) || cost_tmp < reached[id].cost)
            {
                reached[id].id = id;

                reached[id].cost = cost_tmp;

                reached[id].parent_id = vertex_cur.id;

                frontier.push(reached[id]);
            }
        }
    }

    return false;
}

void Dij::GenPath(void)
{
    path.push_back(target_id);

    int vertex_id = reached[target_id].parent_id;

    while (vertex_id != -1)
    {
        path.push_back(vertex_id);

        vertex_id = reached[vertex_id].parent_id;
    }

    std::reverse(path.begin(), path.end());

    vertex_index = 1;

    ClearTargetMsg();

    for (auto &&v : path)
    {
        std::cout << v << ' ';
    }
    
    std::cout << std::endl;

    path_msg.data.clear();
    
    for (auto &&i : path)
    {
        path_msg.data.push_back(i);
    }
}

void Dij::PubPath(void)
{
    PathPuber.publish(path_msg);
}

void Dij::PubTarget(TopoMap& topoMap)
{
    if (update_flag)
    {
        if (vertex_index == path.size() - 1)
        {
            ROS_INFO_ONCE("At the end of path");

            ClearTargetMsg();
        }
        else
        {
            std::cout << "From " << path[vertex_index] << " to " << path[vertex_index + 1] << std::endl;

            // std::vector<double> target_vec = topoMap.get_targetPoint(path[vertex_index], path[vertex_index + 1]);
            std::vector<perception::WayPoint> target_vec = topoMap.get_wayPoints(path[vertex_index], path[vertex_index + 1]);

            ++vertex_index;

            update_flag = false;
            
            // target_vec[2] = target_vec[2] * M_PI / 180;

            // while (target_vec[2] <= -M_PI)
            // {
            //     target_vec[2] += 2 * M_PI;
            // }
            
            // while (target_vec[2] > M_PI)
            // {
            //     target_vec[2] -= 2 * M_PI;
            // }
            
            // target_msg.pose.pose.position.x = target_vec[0];

            // target_msg.pose.pose.position.y = target_vec[1];

            // target_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_vec[2]);   
            
            target_msg.header.stamp = ros::Time::now();

            target_msg.poses.clear();

            for (size_t i = 0; i < target_vec.size(); i++)
            {
                geometry_msgs::PoseStamped node_msg;
                
                node_msg.pose.position.x = target_vec[i].x;
                node_msg.pose.position.y = target_vec[i].y;

                double theta = target_vec[i].theta;
                theta = theta * M_PI/180;
                while (theta <= -M_PI)
                {
                    theta += 2*M_PI;
                }
                while (theta > M_PI)
                {
                    theta -= 2*M_PI;
                }
                
                node_msg.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

                target_msg.poses.push_back(node_msg);
            }
        }
    }

    TargetPuber.publish(target_msg);
}

void Dij::ClearTargetMsg(void)
{
    target_msg.header.stamp = ros::Time::now();

    // target_msg.pose.pose.position.x = 0;

    // target_msg.pose.pose.position.y = 0;

    // target_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);

    target_msg.poses.clear();
}

} // namespace dij
