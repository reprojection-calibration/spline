#include "so3_spline.hpp"

#include "constants.hpp"
#include "lie.hpp"
#include "r3_spline.hpp"  // REMOVE AND USE COMMON GENERIC IMPLEMENTATION
#include "types.hpp"
#include "utilities.hpp"

namespace reprojection_calibration::spline {

So3Spline::So3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns)
    : time_handler_{t0_ns, delta_t_ns, constants::k} {}

std::optional<Eigen::Matrix3d> So3Spline::Evaluate(uint64_t const t_ns) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(knots_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    static MatrixKK const M{CumulativeBlendingMatrix(constants::k)};  // Static means it only evaluates once :)
    // TODO(Jack): Use common generic method one! Pay attention to how we use the constants here though! If we will
    // always be the same dimension for both position and rotation maybe that simplifies things.
    VectorK const u{r3Spline::CalculateU(u_i, DerivativeOrder::Null)};
    VectorK const weight{M * u};

    // TODO(Jack): Can we replace this all with a std::accumulate call?
    Eigen::Matrix3d rotation{knots_[i]};
    for (int j{0}; j < (constants::k - 1); ++j) {
        Eigen::Vector3d const delta_j{Delta(knots_[i + j], knots_[i + j + 1])};
        rotation *= Exp(weight[j + 1] * delta_j);
    }

    return rotation;
}

// TODO(Jack): We could return matrices from all these by returning skew symmetric matrices, but I am not sure if that
// makes sense yet :)
std::optional<Eigen::Vector3d> So3Spline::EvaluateVelocity(uint64_t const t_ns) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(knots_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    static MatrixKK const M{CumulativeBlendingMatrix(constants::k)};  // Static means it only evaluates once :)
    VectorK const u0{r3Spline::CalculateU(u_i, DerivativeOrder::Null)};
    VectorK const weight0{M * u0};

    VectorK const u1{r3Spline::CalculateU(u_i, DerivativeOrder::First)};
    VectorK const weight1{M * u1 / std::pow(time_handler_.delta_t_ns_, static_cast<int>(DerivativeOrder::First))};

    // TODO(Jack): Can we replace this all with a std::accumulate call?
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    for (int j{0}; j < (constants::k - 1); ++j) {
        Eigen::Vector3d const delta_j{Delta(knots_[i + j], knots_[i + j + 1])};
        Eigen::Matrix3d const rotation{Exp(-weight0[j + 1] * delta_j)};

        velocity = rotation * velocity;
        velocity += weight1[j + 1] * delta_j;
    }

    return velocity;
}

std::optional<Eigen::Vector3d> So3Spline::EvaluateAcceleration(uint64_t const t_ns) const {
    auto const normalized_position{time_handler_.SplinePosition(t_ns, std::size(knots_))};
    if (not normalized_position.has_value()) {
        return std::nullopt;
    }
    auto const [u_i, i]{normalized_position.value()};

    static MatrixKK const M{CumulativeBlendingMatrix(constants::k)};  // Static means it only evaluates once :)
    VectorK const u0{r3Spline::CalculateU(u_i, DerivativeOrder::Null)};
    VectorK const weight0{M * u0};

    VectorK const u1{r3Spline::CalculateU(u_i, DerivativeOrder::First)};
    VectorK const weight1{M * u1 / std::pow(time_handler_.delta_t_ns_, static_cast<int>(DerivativeOrder::First))};

    VectorK const u2{r3Spline::CalculateU(u_i, DerivativeOrder::Second)};
    VectorK const weight2{M * u2 / std::pow(time_handler_.delta_t_ns_, static_cast<int>(DerivativeOrder::Second))};

    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    for (int j{0}; j < (constants::k - 1); ++j) {
        Eigen::Vector3d const delta_j{Delta(knots_[i + j], knots_[i + j + 1])};
        Eigen::Matrix3d const rotation{Exp(-weight0[j + 1] * delta_j)};

        velocity = rotation * velocity;
        Eigen::Vector3d const vel_current{weight1[j + 1] * delta_j};
        velocity += vel_current;

        acceleration = rotation * acceleration;
        acceleration += weight2[j + 1] * delta_j + velocity.cross(vel_current);
    }

    return acceleration;
}

Eigen::Vector3d So3Spline::Delta(Eigen::Matrix3d const& R_0, Eigen::Matrix3d const& R_1) const {
    return Log(R_0.inverse() * R_1);
}

}  // namespace reprojection_calibration::spline
