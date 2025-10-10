#include "lie.hpp"

#include <gtest/gtest.h>

using namespace reprojection_calibration::spline;

// Testing Util
bool IsRotation(Eigen::Matrix3d const& R) {
    Eigen::Matrix3d const RRT{R * R.transpose()};  // For rotations R^T = R^-1
    bool const is_orthogonal{(RRT - Eigen::Matrix3d::Identity()).norm() < 1e-10};

    double const D{R.determinant()};
    bool const is_proper{(D - 1) < std::numeric_limits<double>::epsilon()};  // Determinant is positive one

    return is_orthogonal and is_proper;
}

TEST(Lie, TestExp) {
    // NOTE(Jack): These values in the tests are more or less just intuitive heuristic values - if anyone has a better
    // idea to prove that our Exp() implementation works please feel free to contribute that :)
    Eigen::Vector3d const phi{Eigen::Vector3d(0, 0, 0)};
    Eigen::Matrix3d const R{Exp(phi)};
    EXPECT_TRUE(IsRotation(R));
    EXPECT_TRUE(R.isApprox(Eigen::Matrix3d::Identity()));

    Eigen::Vector3d const phi_1{Eigen::Vector3d(M_PI, 0, 0)};
    Eigen::Matrix3d const R_1{Exp(phi_1)};
    EXPECT_TRUE(IsRotation(R_1));
    EXPECT_TRUE(R_1.diagonal().isApprox(Eigen::Vector3d{1, -1, -1}));

    Eigen::Vector3d const phi_2{Eigen::Vector3d(0, M_PI, 0)};
    Eigen::Matrix3d const R_2{Exp(phi_2)};
    EXPECT_TRUE(IsRotation(R_2));
    EXPECT_TRUE(R_2.diagonal().isApprox(Eigen::Vector3d{-1, 1, -1}));

    Eigen::Vector3d const phi_3{Eigen::Vector3d(0, 0, M_PI)};
    Eigen::Matrix3d const R_3{Exp(phi_3)};
    EXPECT_TRUE(IsRotation(R_3));
    EXPECT_TRUE(R_3.diagonal().isApprox(Eigen::Vector3d{-1, -1, 1}));

    Eigen::Matrix3d const sum{R_1 * R_2 * R_3};
    EXPECT_TRUE(IsRotation(sum));
    EXPECT_TRUE(sum.isApprox(
        Eigen::Matrix3d::Identity()));  // Are the coordinates that we fed in above in essence a singularity?
}

TEST(Lie, TestLog) {
    // TODO(Jack): I really wanted to do a symmetric test to the test I did above for Exp(), but unfortunately it looks
    // like our Log() function does not handle edge cases properly. For example I cannot make a rotation matrix with
    // diagonal {1, -1, -1} and get back {M_PI, 0, 0} like I had hoped. Furthermore there is a a big in the Log()
    // function when the angle is equal to pi, the sin of it will be zero and we will get a divide by zero :( This needs
    // to be looked at closely!

    Eigen::Vector3d const phi{Eigen::Vector3d{0, 0, 0}};
    Eigen::Vector3d const phi_processed{Log(Exp(phi))};  // Log and Exp are inverse of one another
    EXPECT_TRUE(phi.isApprox(phi_processed));            // Check that we get back what we put in

    Eigen::Vector3d const phi_1{Eigen::Vector3d{0.1, 0.2, 0.3}};  // Random value
    Eigen::Vector3d const phi_1_processed{Log(Exp(phi_1))};
    EXPECT_TRUE(phi_1.isApprox(phi_1_processed));

    Eigen::Vector3d const phi_2{Eigen::Vector3d{-0.1, -0.2, -0.3}};
    Eigen::Vector3d const phi_2_processed{Log(Exp(phi_2))};
    EXPECT_TRUE(phi_2.isApprox(phi_2_processed));
}

TEST(Lie, TestHatOperators) {
    Eigen::Vector3d const a{Eigen::Vector3d(1, 2, 3)};

    Eigen::Matrix3d const a_hat{Hat(a)};
    EXPECT_TRUE(a_hat.isApprox(-a_hat.transpose()));  // See https://en.wikipedia.org/wiki/Skew-symmetric_matrix

    Eigen::Vector3d const a_hat_vee{Vee(a_hat)};  // Undo Hat() with Vee() to get back the starting vector
    EXPECT_TRUE(a_hat_vee.isApprox(a));
}
