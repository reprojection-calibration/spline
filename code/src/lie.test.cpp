#include <gtest/gtest.h>

#include <Eigen/Dense>

namespace reprojection_calibration::spline {

Eigen::Matrix3d Hat(Eigen::Vector3d const& a) {
    return Eigen::Matrix3d{{0, -a(2), a(1)}, {a(2), 0, -a(0)}, {-a(1), a(0), 0}};
}

Eigen::Vector3d Vee(Eigen::Matrix3d const& a_hat) { return Eigen::Vector3d{a_hat(2, 1), a_hat(0, 2), a_hat(1, 0)}; }

}  // namespace reprojection_calibration::spline

using namespace reprojection_calibration::spline;

TEST(Lie, TestHatOperators) {
    Eigen::Vector3d const a{Eigen::Vector3d(1, 2, 3)};

    Eigen::Matrix3d const a_hat{Hat(a)};
    EXPECT_TRUE(a_hat.isApprox(-a_hat.transpose()));  // See https://en.wikipedia.org/wiki/Skew-symmetric_matrix

    Eigen::Vector3d const a_hat_vee{Vee(a_hat)};  // Undo Hat() with Vee() to get back the starting vector
    EXPECT_TRUE(a_hat_vee.isApprox(a));
}
