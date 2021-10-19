#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>     //直通滤波
#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

void range_filter(pcl::PointCloud<pcl::PointXYZ> &input_cloud, pcl::PointCloud<pcl::PointXYZ> &output_cloud, const float &min_scan_range_, const float min_z);

//将弧度转换到-π~π区间
inline void radianTransform(float &radian);

/**
 * intersectionLocation gets the pose of vehicle at intersections
 * @param pose pose of vehicle
 * @param target_cloud target_cloud, get from topoMap pcd
 * @param input_cloud real-time lidar cloud
 * @param viewer visualization
 */
void intersectionLocation(std::vector<float> &pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::visualization::PCLVisualizer &viewer);
