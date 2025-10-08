#include "utilities.hpp"

namespace reprojection_calibration::spline {

// Factorial based implementation is not the fastest, but we are dealing with small values (?) so we can afford it for
// the sake of clarity https://en.wikipedia.org/wiki/Binomial_coefficient
uint64_t BinomialCoefficient(uint64_t const n, uint64_t const k) {
    return Factorial(n) / (Factorial(k) * Factorial(n - k));
}

uint64_t Factorial(uint64_t const n) {
    uint64_t f{1};
    for (uint64_t i{1}; i <= n; ++i) {
        f *= i;
    }

    return f;
}

}  // namespace reprojection_calibration::spline