#include "utilities.hpp"

namespace reprojection_calibration::spline {

uint64_t Factorial(uint64_t const n) {
    uint64_t f{1};
    for (uint64_t i{1}; i <= n; ++i) {
        f *= i;
    }

    return f;
}

}  // namespace reprojection_calibration::spline