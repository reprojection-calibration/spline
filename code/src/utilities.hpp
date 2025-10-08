#pragma once

#include <Eigen/Dense>
#include <vector>

namespace reprojection_calibration::spline {

double AlternatingSum(int const n, double const increment_1, double const increment_2);

}  // namespace reprojection_calibration::spline