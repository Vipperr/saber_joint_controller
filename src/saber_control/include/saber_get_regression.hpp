#ifndef UR5E_GET_REGRESSION_HPP
#define UR5E_GET_REGRESSION_HPP
#include <rclcpp/rclcpp.hpp>
#include <math.h>

void saber_get_regression(double* H, const array<double, 6>& q, const array<double, 6>& dq, const array<double, 6>& ddq, double g);

#endif