#pragma once

#include <Eigen/Dense>
#include <tuple>

namespace reprojection_calibration::spline {

// Calculates what [1] calls "u" - "normalized time elapsed since start of the segment" - see the second paragraph in
// section 4.2 Matrix Representation. In addition to the normalized segment time u we also return the segment index i as
// this is useful information for error checking later and follows the "law of useful return" principle :)
std::tuple<double, int> NormalizedSegmentTime(uint64_t const t0_ns, uint64_t const t_ns, uint64_t const delta_t_ns);

// TODO(Jack): How can we also make this calculate the derivatives?
Eigen::VectorXd TimePolynomial(int const k, double const u);

Eigen::MatrixXd BlendingMatrix(int const k);

Eigen::MatrixXd CumulativeBlendingMatrix(int const k);

int BinomialCoefficient(int const n, int const k);

int Factorial(int const n);

}  // namespace reprojection_calibration::spline