#include "utilities.hpp"

#include <vector>

namespace reprojection_calibration::spline {

double AlternatingSum(int const n, double const increment_1, double const increment_2) {
    double sum{0};
    for (int i{0}; i < n; ++i) {
        sum += (i % 2 == 0) ? increment_1 : increment_2;
    }

    return sum;
}

}  // namespace reprojection_calibration::spline