#include "utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection_calibration::spline;

// Reference [1] Efficient Derivative Computation for B-Splines on Lie Groups
// Reference [2] Spline Fusion: A continuous-time representation for visual-inertial fusion with application to rolling
// shutter cameras

Eigen::MatrixXd BlendingMatrix(int const k) {
    auto result{Eigen::MatrixXd::Zero(k, k).eval()};

    for (int s{0}; s < k; ++s) {
        for (int n{0}; n < k; ++n) {
            double sum_s_n{0};
            for (int l{s}; l < k; ++l) {
                sum_s_n += std::pow(-1, l - s) * BinomialCoefficient(k, l - s) * std::pow(k - 1 - l, k - 1 - n);
            }
            result(s, n) = BinomialCoefficient(k - 1, n) * sum_s_n;
        }
    }

    return result / Factorial(k - 1);
}

Eigen::MatrixXd CumulativeBlendingMatrix(int const k) {
    Eigen::MatrixXd const blending_matrix{BlendingMatrix(4)};

    auto result{Eigen::MatrixXd::Zero(k, k).eval()};
    for (int s{0}; s < k; ++s) {
        for (int n{0}; n < k; ++n) {
            // Sum of all elements in column at or below element (l, n)
            double sum_s_n{0};
            for (int l{s}; l < k; ++l) {
                sum_s_n += blending_matrix(l, n);
            }
            result(s, n) = sum_s_n;
        }
    }

    return result;
}

TEST(Utilities, TestBlendingMatrix) {
    Eigen::MatrixXd const blender{BlendingMatrix(4)};
    EXPECT_FLOAT_EQ(blender.norm(), 1.7480147);  // Heuristic

    Eigen::MatrixXd const cumulative_blender{CumulativeBlendingMatrix(4)};
    EXPECT_FLOAT_EQ(cumulative_blender.norm(), 1.6996732);                                     // Heuristic
    EXPECT_TRUE(cumulative_blender.row(0).isApprox(Eigen::Vector4d{1, 0, 0, 0}.transpose()));  // Eqn. 20 from [1]

    // Ground-truth value comes from the C matrix at the top of page five in [2]
    Eigen::Matrix4d gt_cumulative_blender{{6, 0, 0, 0}, {5, 3, -3, 1}, {1, 3, 3, -2}, {0, 0, 0, 1}};
    gt_cumulative_blender /= 6;
    EXPECT_TRUE(cumulative_blender.isApprox(gt_cumulative_blender));
}

TEST(Utilities, TestBinomialCoefficient) {
    // Wiki: " where it gives the number of ways, disregarding order, that k objects can be chosen from among n objects"
    EXPECT_EQ(BinomialCoefficient(0, 0), 1);
    EXPECT_EQ(BinomialCoefficient(10, 0), 1);
    EXPECT_EQ(BinomialCoefficient(5, 5), 1);
    EXPECT_EQ(BinomialCoefficient(4, 2), 6);  // Six ways to choose two elements from {1, 2, 3, 4}, namely {1, 2}, {1,
                                              // 3}, {1, 4}, {2, 3}, {2, 4} and {3, 4}.
}

TEST(Utilities, TestFactorial) {
    EXPECT_EQ(Factorial(0), 1);
    EXPECT_EQ(Factorial(1), 1);
    EXPECT_EQ(Factorial(2), 2);
    EXPECT_EQ(Factorial(3), 6);
}