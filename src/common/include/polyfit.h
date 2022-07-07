#pragma once

#include <Eigen/Dense>
#include <math.h>
#include <algorithm>

/**
 * 车道线最小二乘法拟合
 * @param xvals x坐标向量
 * @param yvals y坐标向量
 * @param order 拟合阶数
 */
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, const int order);

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

Eigen::VectorXd offsetFit(const Eigen::VectorXd coeffs, const double offset);