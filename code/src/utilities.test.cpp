#include "utilities.hpp"

#include <gtest/gtest.h>

using namespace reprojection_calibration::spline;

TEST(Utilities, TestFactorial) {
    EXPECT_EQ(Factorial(0), 1);
    EXPECT_EQ(Factorial(1), 1);
    EXPECT_EQ(Factorial(2), 2);
    EXPECT_EQ(Factorial(3), 6);
}