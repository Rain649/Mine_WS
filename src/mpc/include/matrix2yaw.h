#pragma once

#include <Eigen/Dense>

double matrix2yaw(Eigen::Matrix3d transformation)
{
    double yaw_sin = (-transformation(0, 1) + transformation(1, 0)) / 2;
    double yaw_cos = (transformation(0, 0) + transformation(1, 1)) / 2;
    double yaw;
    if (yaw_cos == 0)
        yaw = (yaw_sin > 0) ? M_PI / 2 : -M_PI / 2;
    else
    {
        yaw = atan(yaw_sin / yaw_cos);
        if (yaw_cos < 0)
            yaw += (yaw_sin > 0) ? M_PI : -M_PI;
    }
    return yaw;
}