#pragma once

#include <Eigen/Dense>

namespace reprojection_calibration::spline {

Eigen::MatrixXd BlendingMatrix(int const k);

Eigen::MatrixXd CumulativeBlendingMatrix(int const k);

uint64_t BinomialCoefficient(uint64_t const n, uint64_t const k);

uint64_t Factorial(uint64_t const n);

}  // namespace reprojection_calibration::spline