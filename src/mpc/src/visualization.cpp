#include <visualization.h>
#include "mpc_param.h"

std::string mapFrame_id = "localMap";
std::string baseLink_id = "vehicle_base_link";

visualization_msgs::Marker showVehicle()
{
    visualization_msgs::Marker vehicleCube;
    vehicleCube.header.frame_id = baseLink_id;
    vehicleCube.header.stamp = ros::Time::now();
    vehicleCube.ns = "cube";
    vehicleCube.action = visualization_msgs::Marker::ADD;
    vehicleCube.pose.orientation.w = 1.0;
    vehicleCube.id = 0;
    vehicleCube.type = visualization_msgs::Marker::CUBE;
    vehicleCube.scale.x = 4;
    vehicleCube.scale.y = 1.9;
    vehicleCube.scale.z = 2;
    // 红色
    vehicleCube.color.r = 1.0;
    vehicleCube.color.a = 1;

    vehicleCube.pose.position.x = 0;
    vehicleCube.pose.position.y = 0;
    vehicleCube.pose.position.z = 0;
    vehicleCube.pose.orientation.w = 1;
    vehicleCube.pose.orientation.x = 0;
    vehicleCube.pose.orientation.y = 0;
    vehicleCube.pose.orientation.z = 0;

    return vehicleCube;
}

visualization_msgs::Marker showVehicle(const VehicleState &input)
{
    visualization_msgs::Marker vehicleCube;
    vehicleCube.header.frame_id = mapFrame_id;
    vehicleCube.header.stamp = ros::Time::now();
    vehicleCube.ns = "cube";
    vehicleCube.action = visualization_msgs::Marker::ADD;
    vehicleCube.pose.orientation.w = 1.0;
    vehicleCube.id = 0;
    vehicleCube.type = visualization_msgs::Marker::CUBE;
    vehicleCube.scale.x = 4;
    vehicleCube.scale.y = 1.9;
    vehicleCube.scale.z = 2;
    // 红色
    vehicleCube.color.r = 1.0;
    vehicleCube.color.a = 1;

    Eigen::AngleAxisd yawAngle(input.yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond quaternion;
    quaternion = yawAngle;

    vehicleCube.pose.position.x = input.x;
    vehicleCube.pose.position.y = input.y;
    vehicleCube.pose.position.z = 0;
    vehicleCube.pose.orientation.w = quaternion.w();
    vehicleCube.pose.orientation.x = quaternion.x();
    vehicleCube.pose.orientation.y = quaternion.y();
    vehicleCube.pose.orientation.z = quaternion.z();

    return vehicleCube;
}
visualization_msgs::Marker showReferencePath(const std::vector<Path_struct> &input)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = mapFrame_id;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "referencePath";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 1;
    // 蓝色
    line_strip.color.b = 1.0;
    line_strip.color.a = 0.5;
    for (size_t i = 0; i < input.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = input[i].x;
        p.y = input[i].y;
        p.z = 0;
        line_strip.points.push_back(p);
    }

    return line_strip;
}
visualization_msgs::Marker showPredictedTrajectory(const std::vector<Path_struct> &input)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = baseLink_id;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "predictedTrajectory";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 2;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 1;
    // 绿色
    line_strip.color.g = 1.0;
    line_strip.color.a = 0.5;
    for (size_t i = 0; i < input.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = input[i].x;
        p.y = input[i].y;
        p.z = 0;
        line_strip.points.push_back(p);
    }
    return line_strip;
}
visualization_msgs::Marker showReferencePoints(const std::vector<Path_struct> &input)
{
    visualization_msgs::Marker points;
    points.header.frame_id = mapFrame_id;
    points.header.stamp = ros::Time::now();
    points.ns = "referencePoints";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 3;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = points.scale.y = 1.0;
    // 橙色
    points.color.r = 1.0;
    points.color.g = 165 / 255;
    points.color.a = 0.8;
    for (size_t i = 0; i < input.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = input[i].x;
        p.y = input[i].y;
        p.z = 0;
        points.points.push_back(p);
    }
    return points;
}
visualization_msgs::Marker showPolyLine(const std::vector<Path_struct> &input)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = baseLink_id;
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "polyLine";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;
    line_strip.id = 4;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 1;
    // Line strip 是黄色
    line_strip.color.r = line_strip.color.g = 1.0;
    line_strip.color.a = 0.5;
    for (size_t i = 0; i < input.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = input[i].x;
        p.y = input[i].y;
        p.z = 0;
        line_strip.points.push_back(p);
    }
    return line_strip;
}
