#pragma once

#include <optional>

#include "types_and_constants.hpp"

namespace reprojection_calibration::spline {

// NOTE(Jack): It is a unfiform spline which presupposes that all added knots correspond to specific evenly spaced
// times.
class r3Spline {
   public:
    r3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns);

    // TODO(Jack): use enum and static cast instead of int
    std::optional<VectorD> Evaluate(uint64_t const t_ns,
                                    DerivativeOrder const derivative = DerivativeOrder::Zero) const;

    // TODO(Jack): Let us consider what benefit we would get from making this private at some later point
    std::vector<VectorD> knots_;  // A.k.a. "control points"

   private:
    uint64_t t0_ns_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection_calibration::spline
