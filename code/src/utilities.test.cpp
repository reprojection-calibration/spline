#include "utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection_calibration::spline;

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

TEST(Utilities, TestBlendingMatrix) {
    std::cout << BlendingMatrix(4) << std::endl;
    EXPECT_EQ(0, 1);
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