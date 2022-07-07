#pragma once

#include <Eigen/Dense>
#include "vehicleState.h"
#include "path_struct.h"
#include <visualization_msgs/Marker.h>

//车辆Cube
visualization_msgs::Marker showVehicle();
//车辆Cube
visualization_msgs::Marker showVehicle(const VehicleState &input);
//参考路径
visualization_msgs::Marker showReferencePath(const std::vector<Path_struct> &input);
//车辆预测路径
visualization_msgs::Marker showPredictedTrajectory(const std::vector<Path_struct> &input);
//参考点
visualization_msgs::Marker showReferencePoints(const std::vector<Path_struct> &input);
//拟合曲线
visualization_msgs::Marker showPolyLine(const std::vector<Path_struct> &input);