#pragma once

#include <Eigen/Dense>

namespace reprojection_calibration::spline {

// Calculates what [1] calls "u" - "normalized time elapsed since start of the segment" - see the second paragraph in
// section 4.2 Matrix Representation
double SegmentTime(double const t0_ns, double const t_ns, double const delta_t_ns);

// TODO(Jack): How can we also make this calculate the derivatives?
Eigen::VectorXd TimePolynomial(int const k, double const u);

Eigen::MatrixXd BlendingMatrix(int const k);

Eigen::MatrixXd CumulativeBlendingMatrix(int const k);

int BinomialCoefficient(int const n, int const k);

int Factorial(int const n);

}  // namespace reprojection_calibration::spline