#include "r3_spline.hpp"

#include "constants.hpp"
#include "utilities.hpp"

namespace reprojection_calibration::spline {

r3Spline::r3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns) : t0_ns_{t0_ns}, delta_t_ns_{delta_t_ns} {}

std::optional<VectorD> r3Spline::Evaluate(uint64_t const t_ns, DerivativeOrder const derivative) const {
    auto const [u_i, i]{NormalizedSegmentTime(t0_ns_, t_ns, delta_t_ns_)};

    // From reference [1] - "At time t in [t_i, t_i+1) the value of p(t) only depends on the control points p_i,
    // p_i+1, ..., p_i+k-1" - See the start of the second paragraph in section 4.2 Matrix Representation.
    if (std::size(knots_) < static_cast<size_t>(i + constants::k)) {
        return std::nullopt;
    }

    MatrixDK const P{Eigen::Map<const MatrixDK>(knots_[i].data(), constants::d, constants::k)};
    static MatrixKK const M{BlendingMatrix(constants::k)};  // Static means it only evaluates once :)

    // We are constructing the column vectors that we multiply by C as found at the top of page five in [2] - this
    // construction depends on which derivative we are evaluating the spline at.
    static MatrixKK const polynomial_coefficients{PolynomialCoefficients(constants::k)};
    int const derivative_order{static_cast<int>(derivative)};

    // TODO(Jack): This is the only place the derivative order shows up, there has to be a way to better abstract this
    // code and this function in general!
    VectorK const u{polynomial_coefficients.row(derivative_order).transpose().array() *
                    TimePolynomial(constants::k, u_i, derivative_order).array() /
                    std::pow(delta_t_ns_, derivative_order)};

    return (P * M * u);
}

}  // namespace reprojection_calibration::spline
