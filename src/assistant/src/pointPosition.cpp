#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

#include <math.h>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdio.h>
#include <stdlib.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>

#include <sensor_msgs/PointCloud2.h>

#include <thread>

#include "path_struct.h"

using namespace visualization_msgs;

std::vector<Path_struct> path;
pcl::PointCloud<pcl::PointXYZ> inputCloud;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
interactive_markers::MenuHandler menu_handler;
// %EndTag(vars)%
// %Tag(Box)%
Marker makeBox(InteractiveMarker &msg)
{
    Marker marker;

    marker.type = Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
}

InteractiveMarkerControl &makeBoxControl(InteractiveMarker &msg)
{
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(frameCallback)%
void frameCallback(const ros::TimerEvent &)
{
    static uint32_t counter = 0;

    static tf::TransformBroadcaster br;

    tf::Transform t;

    ros::Time time = ros::Time::now();

    t.setOrigin(tf::Vector3(0.0, 0.0, sin(float(counter) / 140.0) * 2.0));
    t.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));
    br.sendTransform(tf::StampedTransform(t, time, "base_link", "moving_frame"));

    t.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    t.setRotation(tf::createQuaternionFromRPY(0.0, float(counter) / 140.0, 0.0));
    br.sendTransform(tf::StampedTransform(t, time, "base_link", "rotating_frame"));

    counter++;
}
// %EndTag(frameCallback)%

// %Tag(processFeedback)%
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
      << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if (feedback->mouse_point_valid)
    {
        mouse_point_ss << " at " << feedback->mouse_point.x
                       << ", " << feedback->mouse_point.y
                       << ", " << feedback->mouse_point.z
                       << " in frame " << feedback->header.frame_id;
    }

    switch (feedback->event_type)
    {
    case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
        ROS_INFO_STREAM(s.str() << ": button click" << mouse_point_ss.str() << ".");
        break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        ROS_INFO_STREAM(s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << ".");
        break;

    case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        ROS_INFO_STREAM(s.str() << ": pose changed"
                                << "\nposition = "
                                << feedback->pose.position.x
                                << ", " << feedback->pose.position.y
                                << ", " << feedback->pose.position.z
                                << "\norientation = "
                                << feedback->pose.orientation.w
                                << ", " << feedback->pose.orientation.x
                                << ", " << feedback->pose.orientation.y
                                << ", " << feedback->pose.orientation.z
                                << "\nframe: " << feedback->header.frame_id
                                << " time: " << feedback->header.stamp.sec << "sec, "
                                << feedback->header.stamp.nsec << " nsec");
        break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_INFO_STREAM(s.str() << ": mouse down" << mouse_point_ss.str() << ".");
        break;

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    {
        ROS_INFO_STREAM(s.str() << ": mouse up" << mouse_point_ss.str() << ".");
        path.push_back(Path_struct(feedback->pose.position.x, feedback->pose.position.y));
        break;
    }
    }

    server->applyChanges();
}
// %EndTag(processFeedback)%

// %Tag(alignMarker)%
void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
    geometry_msgs::Pose pose = feedback->pose;

    // pose.position.x = round(pose.position.x - 0.5) + 0.5;
    // pose.position.y = round(pose.position.y - 0.5) + 0.5;

    ROS_INFO_STREAM(feedback->marker_name << ":"
                                          << " aligning position = "
                                          << feedback->pose.position.x
                                          << ", " << feedback->pose.position.y
                                          << ", " << feedback->pose.position.z
                                          << " to "
                                          << pose.position.x
                                          << ", " << pose.position.y
                                          << ", " << pose.position.z);

    server->setPose(feedback->marker_name, pose);
    server->applyChanges();
}
// %EndTag(alignMarker)%

double rand(double min, double max)
{
    double t = (double)rand() / (double)RAND_MAX;
    return min + t * (max - min);
}

void saveMarker(InteractiveMarker int_marker)
{
    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);
}

//////////////////////////////////////////////////////////////////////////////////

// %Tag(ChessPiece)%
void makeChessPieceMarker(const tf::Vector3 &position)
{
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    int_marker.name = "chess_piece";
    int_marker.description = "Chess Piece\n(2D Move + Alignment)";

    InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
    int_marker.controls.push_back(control);

    // make a box which also moves in the plane
    control.markers.push_back(makeBox(int_marker));
    control.always_visible = true;
    int_marker.controls.push_back(control);

    // we want to use our special callback function
    server->insert(int_marker);
    server->setCallback(int_marker.name, &processFeedback);

    // set different callback for POSE_UPDATE feedback
    server->setCallback(int_marker.name, &alignMarker, visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
}
// %EndTag(ChessPiece)%

void commandProcess()
{
    std::string command;
    while (ros::ok())
    {
        command.clear();
        ROS_INFO("\033[1;32m---->\033[0m Please Drag The Intersection Information File In:");
        std::getline(std::cin, command);
        if (command == "")
            continue;
        else if (command == "c")
        {
            path.clear();
            continue;
        }
        else if (command == "z" && !path.empty())
        {
            path.pop_back();
            continue;
        }
        else if (command == "w")
        {
            std::ofstream ofs;
            ofs.open("src/common/temp/path_temp.txt", std::ios::out);
            ofs.clear();
            for (size_t i = 0; i < path.size(); ++i)
            {
                ofs << std::setprecision(3) << "     - ["
                    << path[i].x << "," << path[i].y << ","
                    << "0]";
                if (i != path.size() - 1)
                {
                    ofs << std::endl;
                }
            }

            ofs.close();
        }
        else if (command == "ww")
        {
            std::ofstream ofs;
            ofs.open("src/common/temp/path_temp.txt", std::ios::out);
            ofs.clear();
            if (!path.empty())
                ofs << std::setprecision(3) << std::endl
                    << "     startPoint: ["
                    << path[path.size() - 1].x << "," << path[path.size() - 1].y << ","
                    << "0]";

            ofs.close();
        }
        else if (command == "q")
        {
            break;
        }
        else if (pcl::io::loadPCDFile<pcl::PointXYZ>(command.substr(1, command.length() - 3), inputCloud) == -1)
        {
            PCL_ERROR("Couldn't read file %s \n", (command.substr(1, command.length() - 3)).c_str());
            continue;
        }
        ROS_INFO("\033[1;32m---->\033[0m Read PCD File Successfully");
    }
}

visualization_msgs::Marker showReferencePath(const std::vector<Path_struct> &input)
{
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "base_link";
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
void visualization()
{
    ros::NodeHandle nh("~");
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("cloud", 1);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    sensor_msgs::PointCloud2 output;
    while (ros::ok())
    {
        marker_pub.publish(showReferencePath(path));

        //发布点云
        inputCloud.header.frame_id = "base_link";
        toROSMsg(inputCloud, output);
        cloud_pub.publish(output);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointPathSave");
    ros::NodeHandle nh("~");

    ROS_INFO("\033[1;32m---->\033[0m Point Position Started.");

    // create a timer to update the published transforms
    ros::Timer frame_timer = nh.createTimer(ros::Duration(0.01), frameCallback);

    server.reset(new interactive_markers::InteractiveMarkerServer("pointPathSave", "", false));

    ros::Duration(0.1).sleep();

    menu_handler.insert("First Entry", &processFeedback);
    menu_handler.insert("Second Entry", &processFeedback);
    interactive_markers::MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert("Submenu");
    menu_handler.insert(sub_menu_handle, "First Entry", &processFeedback);
    menu_handler.insert(sub_menu_handle, "Second Entry", &processFeedback);

    tf::Vector3 position;
    position = tf::Vector3(0, 0, 0);
    makeChessPieceMarker(position);

    server->applyChanges();

    std::thread thread_cloud(commandProcess);
    std::thread thread_pathShow(visualization);

    ros::spin();

    thread_cloud.join();
    thread_pathShow.join();

    server.reset();

    return 0;
}