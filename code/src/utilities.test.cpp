#include "utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection_calibration::spline;

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