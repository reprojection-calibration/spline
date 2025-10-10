#include "lie.hpp"

#include <gtest/gtest.h>

namespace reprojection_calibration::spline {}  // namespace reprojection_calibration::spline

using namespace reprojection_calibration::spline;

TEST(Lie, TestHatOperators) {
    Eigen::Vector3d const a{Eigen::Vector3d(1, 2, 3)};

    Eigen::Matrix3d const a_hat{Hat(a)};
    EXPECT_TRUE(a_hat.isApprox(-a_hat.transpose()));  // See https://en.wikipedia.org/wiki/Skew-symmetric_matrix

    Eigen::Vector3d const a_hat_vee{Vee(a_hat)};  // Undo Hat() with Vee() to get back the starting vector
    EXPECT_TRUE(a_hat_vee.isApprox(a));
}
