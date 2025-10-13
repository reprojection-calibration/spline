#include "so3_spline.hpp"

#include "constants.hpp"
#include "lie.hpp"
#include "r3_spline.hpp"  // REMOVE AND USE COMMON GENERIC IMPLEMENTATION
#include "types.hpp"
#include "utilities.hpp"

namespace reprojection_calibration::spline {

So3Spline::So3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns)
    : time_handler_{t0_ns, delta_t_ns, constants::k} {}

std::optional<Eigen::Matrix3d> So3Spline::Evaluate(uint64_t const t_ns, DerivativeOrder const derivative) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(knots_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    static MatrixKK const M{CumulativeBlendingMatrix(constants::k)};  // Static means it only evaluates once :)
    // TODO(Jack): Use common generic method one! Pay attention to how we use the constants here though! If we will
    // always be the same dimension for both position and rotation maybe that simplifies things.
    VectorK const u{r3Spline::CalculateU(u_i, derivative)};
    VectorK const weight{M * u};

    // TODO(Jack): Can we replace this all with a std::accumulate call?
    Eigen::Matrix3d result{knots_[i]};
    for (int j{0}; j < (constants::k - 1); ++j) {
        Eigen::Vector3d const delta_j{Delta(knots_[i + j], knots_[i + j + 1])};
        result *= Exp(weight[j + 1] * delta_j);
    }

    return result;
}

Eigen::Vector3d So3Spline::Delta(Eigen::Matrix3d const & R_0, Eigen::Matrix3d const& R_1) const {
    return Log(R_0.inverse() * R_1);
}

}  // namespace reprojection_calibration::spline
