#include "lie.hpp"

#include <cmath>

namespace reprojection_calibration::spline {

Eigen::Matrix3d Exp(Eigen::Vector3d const& phi) {
    double const angle{phi.norm()};

    if (angle < 1e-6) {
        // use first order taylor expansion when phi is small
        return Eigen::Matrix3d::Identity() + Hat(phi);
    }

    Eigen::Vector3d const axis{phi / angle};
    double const cos{std::cos(angle)};
    double const sin{std::sin(angle)};

    return ((cos * Eigen::Matrix3d::Identity()) + ((1.0 - cos) * axis * axis.transpose()) + (sin * Hat(axis)));
}

Eigen::Vector3d Log(Eigen::Matrix3d const& R) {
    double cos{(0.5 * R.trace()) - 0.5};
    cos = std::clamp(cos, -1.0, 1.0);

    double const angle{std::acos(cos)};

    if (angle < 1e-6) {
        // use first order taylor expansion when angle is small
        return Vee(R - Eigen::Matrix3d::Identity());
    }

    // ERROR(Jack): What about when angle is equal to pi? Then we risk a division by one unless we adjust the condition
    // above?
    return Vee((0.5 * angle / std::sin(angle)) * (R - R.transpose()));
}

Eigen::Matrix3d Hat(Eigen::Vector3d const& a) {
    return Eigen::Matrix3d{{0, -a(2), a(1)}, {a(2), 0, -a(0)}, {-a(1), a(0), 0}};
}

Eigen::Vector3d Vee(Eigen::Matrix3d const& a_hat) { return Eigen::Vector3d{a_hat(2, 1), a_hat(0, 2), a_hat(1, 0)}; }

}  // namespace reprojection_calibration::spline
