#include "lie.hpp"

#include <gtest/gtest.h>

// Testing Util

bool IsRotation(Eigen::Matrix3d const& R) {
    Eigen::Matrix3d const RRT{R * R.transpose()};  // For rotations R^T = R^-1
    bool const is_orthogonal{(RRT - Eigen::Matrix3d::Identity()).norm() < 1e-10};

    double const D{R.determinant()};
    bool const is_proper{(D - 1) < std::numeric_limits<double>::epsilon()};  // Determinant is positive one

    return is_orthogonal and is_proper;
}

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

}  // namespace reprojection_calibration::spline

using namespace reprojection_calibration::spline;

TEST(Lie, TestExp) {
    // NOTE(Jack): These values in the tests are more or less just intuitive heuristic values - if anyone has a better
    // idea to prove that our Exp() implementation works please feel free to contribute that :)
    Eigen::Vector3d const a{Eigen::Vector3d(0, 0, 0)};
    Eigen::Matrix3d const exp_a{Exp(a)};
    EXPECT_TRUE(IsRotation(exp_a));
    EXPECT_TRUE(exp_a.isApprox(Eigen::Matrix3d::Identity()));

    Eigen::Vector3d const a1{Eigen::Vector3d(M_PI, 0, 0)};
    Eigen::Matrix3d const exp_a1{Exp(a1)};
    EXPECT_TRUE(IsRotation(exp_a1));
    EXPECT_TRUE(exp_a1.diagonal().isApprox(Eigen::Vector3d{1, -1, -1}));

    Eigen::Vector3d const a2{Eigen::Vector3d(0, M_PI, 0)};
    Eigen::Matrix3d const exp_a2{Exp(a2)};
    EXPECT_TRUE(IsRotation(exp_a2));
    EXPECT_TRUE(exp_a2.diagonal().isApprox(Eigen::Vector3d{-1, 1, -1}));

    Eigen::Vector3d const a3{Eigen::Vector3d(0, 0, M_PI)};
    Eigen::Matrix3d const exp_a3{Exp(a3)};
    EXPECT_TRUE(IsRotation(exp_a3));
    EXPECT_TRUE(exp_a3.diagonal().isApprox(Eigen::Vector3d{-1, -1, 1}));

    Eigen::Matrix3d const sum{exp_a1 * exp_a2 * exp_a3};
    EXPECT_TRUE(IsRotation(sum));
    EXPECT_TRUE(sum.isApprox(Eigen::Matrix3d::Identity()));  // Is this in essence a singularity?
}

TEST(Lie, TestLog) {
    // TODO(Jack): I really wanted to do a symmetric test to the test I did above for Exp(), but unfortunately it looks
    // like our Log() function does not handle edge cases properly. For example I cannot make a rotation matrix with
    // diagonal {1, -1, -1} and get back {M_PI, 0, 0} like I had hoped. Furthermore there is a a big in the Log()
    // function when the angle is equal to pi, the sin of it will be zero and we will get a divide by zero :( This needs
    // to be looked at closely!

    Eigen::Vector3d const a{Eigen::Vector3d{0, 0, 0}};
    Eigen::Vector3d const a_processed{Log(Exp(a))};  // Log and Exp are inverse of one another
    EXPECT_TRUE(a.isApprox(a_processed));            // Check that we get back what we put in

    Eigen::Vector3d const a1{Eigen::Vector3d{0.1, 0.2, 0.3}};  // Random value
    Eigen::Vector3d const a1_processed{Log(Exp(a1))};
    EXPECT_TRUE(a1.isApprox(a1_processed));

    Eigen::Vector3d const a2{Eigen::Vector3d{-0.1, -0.2, -0.3}};
    Eigen::Vector3d const a2_processed{Log(Exp(a2))};
    EXPECT_TRUE(a2.isApprox(a2_processed));
}

TEST(Lie, TestHatOperators) {
    Eigen::Vector3d const a{Eigen::Vector3d(1, 2, 3)};

    Eigen::Matrix3d const a_hat{Hat(a)};
    EXPECT_TRUE(a_hat.isApprox(-a_hat.transpose()));  // See https://en.wikipedia.org/wiki/Skew-symmetric_matrix

    Eigen::Vector3d const a_hat_vee{Vee(a_hat)};  // Undo Hat() with Vee() to get back the starting vector
    EXPECT_TRUE(a_hat_vee.isApprox(a));
}
