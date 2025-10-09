#pragma once

#include <Eigen/Dense>

namespace reprojection_calibration::spline {

namespace constants {

// Instead of templating everything like mad we will use this global to parameterize the order of the spline.
inline constexpr int k{4};  // Spline order - note spline "degree" is k-1, so when k=4 it is a cubic spline!
inline constexpr int d{3};  // State dimension for r3 spline

}  // namespace constants

using MatrixDK = Eigen::Matrix<double, constants::d, constants::k>;
using MatrixKK = Eigen::Matrix<double, constants::k, constants::k>;
using VectorD = Eigen::Vector<double, constants::d>;
using VectorK = Eigen::Vector<double, constants::k>;

enum class DerivativeOrder { Zero = 0, First = 1, Second = 2 };

}  // namespace reprojection_calibration::spline