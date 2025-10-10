#include "so3_spline.hpp"

#include "constants.hpp"
#include "lie.hpp"
#include "r3_spline.hpp"  // REMOVE AND USE COMMON GENERIC IMPLEMENTATION
#include "types.hpp"
#include "utilities.hpp"

namespace reprojection_calibration::spline {

So3Spline::So3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns) : t0_ns_{t0_ns}, delta_t_ns_{delta_t_ns} {}

std::optional<Eigen::Matrix3d> So3Spline::Evaluate(uint64_t const t_ns, DerivativeOrder const derivative) const {
    auto const [u_i, i]{NormalizedSegmentTime(t0_ns_, t_ns, delta_t_ns_)};

    if (std::size(knots_) < static_cast<size_t>(i + constants::k)) {
        return std::nullopt;
    }

    static MatrixKK const M{CumulativeBlendingMatrix(constants::k)};  // Static means it only evaluates once :)
    // TODO(Jack): Use common generic method one! Pay attention to how we use the constants here though! If we will
    // alwazs be the same dimension for both position and rotation maybe that simplifies things.
    VectorK const u{r3Spline::CalculateU(u_i, derivative)};

    VectorK const weight{M * u};

    // TODO(Jack): Can we replace this all with a std::accumulate call?
    Eigen::Matrix3d result{knots_[i]};
    for (int j{0}; j < (constants::k - 1); ++j) {
        Eigen::Matrix3d const& p0{knots_[i + j]};
        Eigen::Matrix3d const& p1{knots_[i + j + 1]};

        Eigen::Matrix3d const r01{p0.inverse() * p1};
        Eigen::Vector3d const delta{Log(r01)};
        Eigen::Vector3d const weighted_delta{weight[j + 1] * delta};

        result *= Exp(weighted_delta);
    }

    return result;
}

}  // namespace reprojection_calibration::spline
