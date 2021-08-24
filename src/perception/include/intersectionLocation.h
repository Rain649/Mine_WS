#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <fstream>

// void range_filter(pcl::PointCloud<pcl::PointXYZ> & input_cloud, pcl::PointCloud<pcl::PointXYZ> & output_cloud,const float & min_scan_range_, const float min_z)
// {
//     double r;
//     pcl::PointXYZ p ;
//     for (pcl::PointCloud<pcl::PointXYZ>::const_iterator item = input_cloud.begin();
//         item != input_cloud.end(); ++item)
//     {
//         p.x = (double) item->x;
//         p.y = (double) item->y;
//         p.z = (double) item->z;
//         r = p.x * p.x + p.y * p.y;
//         if (r > min_scan_range_&&p.z >min_z) //filter the point distance lager than min_scan_range and height lower than min_z
//         {
//             output_cloud.push_back(p);
//         }
//     }
// }

/**
 * intersectionLocation gets the pose of vehicle at intersections
 * @param pose pose of vehicle
 * @param target_cloud target_cloud, get from topoMap pcd
 * @param input_cloud real-time lidar cloud
 * @param viewer visualization
 */
void intersectionLocation(std::vector<float> &pose, const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::visualization::PCLVisualizer &viewer){};
