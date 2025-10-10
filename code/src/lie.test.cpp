#include "lie.hpp"

#include <gtest/gtest.h>

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

}  // namespace reprojection_calibration::spline

using namespace reprojection_calibration::spline;

TEST(Lie, TestExp) {
    // NOTE(Jack): These values in the tests are more or less just intuitive heuristic values - if anyone has a better
    // idea to prove that our Exp() implementation works please feel free to contribute that :)
    Eigen::Vector3d const a{Eigen::Vector3d(0, 0, 0)};
    Eigen::Matrix3d const exp_a{Exp(a)};
    EXPECT_TRUE(exp_a.isApprox(Eigen::Matrix3d::Identity()));

    Eigen::Vector3d const a1{Eigen::Vector3d(M_PI, 0, 0)};
    Eigen::Matrix3d const exp_a1{Exp(a1)};
    EXPECT_TRUE(exp_a1.diagonal().isApprox(Eigen::Vector3d{1, -1, -1}));

    Eigen::Vector3d const a2{Eigen::Vector3d(0, M_PI, 0)};
    Eigen::Matrix3d const exp_a2{Exp(a2)};
    EXPECT_TRUE(exp_a2.diagonal().isApprox(Eigen::Vector3d{-1, 1, -1}));

    Eigen::Vector3d const a3{Eigen::Vector3d(0, 0, M_PI)};
    Eigen::Matrix3d const exp_a3{Exp(a3)};
    EXPECT_TRUE(exp_a3.diagonal().isApprox(Eigen::Vector3d{-1, -1, 1}));

    EXPECT_TRUE((exp_a1 * exp_a2 * exp_a3).isApprox(Eigen::Matrix3d::Identity()));
}

TEST(Lie, TestHatOperators) {
    Eigen::Vector3d const a{Eigen::Vector3d(1, 2, 3)};

    Eigen::Matrix3d const a_hat{Hat(a)};
    EXPECT_TRUE(a_hat.isApprox(-a_hat.transpose()));  // See https://en.wikipedia.org/wiki/Skew-symmetric_matrix

    Eigen::Vector3d const a_hat_vee{Vee(a_hat)};  // Undo Hat() with Vee() to get back the starting vector
    EXPECT_TRUE(a_hat_vee.isApprox(a));
}
