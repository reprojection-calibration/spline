#include "lie.hpp"

namespace reprojection_calibration::spline {

Eigen::Matrix3d Hat(Eigen::Vector3d const& a) {
    return Eigen::Matrix3d{{0, -a(2), a(1)}, {a(2), 0, -a(0)}, {-a(1), a(0), 0}};
}

Eigen::Vector3d Vee(Eigen::Matrix3d const& a_hat) { return Eigen::Vector3d{a_hat(2, 1), a_hat(0, 2), a_hat(1, 0)}; }

}  // namespace reprojection_calibration::spline
