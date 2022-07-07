#include <ros/ros.h>
#include <iostream>
#include <pcl/common/distances.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h> //直通滤波
#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <omp.h>

#include "registrationConfig.h"

struct VehicleState
{
    float x;
    float y;
    float yaw;
    float v;
};

/**
 * 将弧度转换到-π~π区间
 * @param radian 传入弧度值
 */
inline void radianTransform(float &radian);

/**
 * intersectionLocation gets the vehicleState of vehicle at intersections
 * @param vehicleState vehicleState of vehicle
 * @param target_cloud target_cloud, get from topoMap pcd
 * @param input_cloud real-time lidar cloud
 * @param registrationConfig registration parameters
 * @param viewer visualization
 */
bool intersectionLocation(VehicleState &vehicleState, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, RegistrationConfig &registrationConfig, pcl::visualization::PCLVisualizer::Ptr viewer);
