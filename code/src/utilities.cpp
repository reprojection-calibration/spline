#include "utilities.hpp"

namespace reprojection_calibration::spline {

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

// Factorial based implementation is not the fastest, but we are dealing with small values (?) so we can afford it for
// the sake of clarity https://en.wikipedia.org/wiki/Binomial_coefficient#Computing_the_value_of_binomial_coefficients
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