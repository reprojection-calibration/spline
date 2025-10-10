#pragma once

#include <Eigen/Dense>

namespace reprojection_calibration::spline {

Eigen::Matrix3d Exp(Eigen::Vector3d const& phi);

Eigen::Vector3d Log(Eigen::Matrix3d const& R);

Eigen::Matrix3d Hat(Eigen::Vector3d const& a);

Eigen::Vector3d Vee(Eigen::Matrix3d const& a_hat);

}  // namespace reprojection_calibration::spline
