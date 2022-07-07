#include "polyfit.h"

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, const int order)
{
    Eigen::VectorXd result;
    if (order >= xvals.size())
        return result;
    Eigen::MatrixXd A(xvals.size(), order + 1);
    for (int i = 0; i < xvals.size(); ++i)
    {
        A(i, 0) = 1.0;
    }
    for (int j = 0; j < xvals.size(); ++j)
    {
        for (int i = 0; i < order; ++i)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }
    auto Q = A.householderQr();
    result = Q.solve(yvals);
    return result;
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{
    double result = 0.0;
    for (int i = 0; i < coeffs.size(); ++i)
    {
        result += coeffs[i] * pow(x, i);
    }
    return result;
}

Eigen::VectorXd offsetFit(const Eigen::VectorXd coeffs, const double offset)
{
    Eigen::VectorXd xvals(24);
    Eigen::VectorXd yvals(24);
    int i = 0;
    double x_min{-5.0}, x_max{20.0}, dx{1.0};
    for (double x = x_min + dx; x < x_max; ++x)
    {
        double x_a = x - dx;
        double y_a = polyeval(coeffs, x_a);
        double x_b = x;
        double y_b = polyeval(coeffs, x_b);
        double x_c = x + dx;
        double y_c = polyeval(coeffs, x_c);

        // double k_ac = (y_c-y_a)/(x_c-x_a);
        // double k_rb = -1/k_ac;
        // double k_rb = (x_a - x_c) / (y_c - y_a);
        double theta = atan2(x_a - x_c, y_c - y_a);
        double x_r = x_b - offset * cos(theta);
        double y_r = y_b - offset * sin(theta);

        // xvals << x_r;
        // yvals << y_r;
        
        xvals[i] = x_r;
        yvals[i] = y_r;
        ++i;
    }
    Eigen::VectorXd result = polyfit(xvals, yvals, 3);
    return result;
}