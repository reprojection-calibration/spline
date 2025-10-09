#include <gtest/gtest.h>

#include "utilities.hpp"

using namespace reprojection_calibration::spline;

namespace reprojection_calibration::spline::constants {

inline constexpr int k{4};  // Spline order

}

// NOTE(Jack): It is a unfiform spline which presupposes that all added knots correspond to specific evenly spaced
// times.
class r3Spline {
   public:
    r3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns) : t0_ns_{t0_ns}, delta_t_ns_{delta_t_ns} {}

    std::optional<Eigen::Vector3d> Evaluate(uint64_t const t_ns) {
        auto const [u, i]{NormalizedSegmentTime(t0_ns_, t_ns, delta_t_ns_)};

        if (std::size(knots_) < static_cast<size_t>(i + constants::k)) {
            return std::nullopt;
        }
    }

    // TODO(Jack): Let us consider what benefit we would get from making this private at some later point
    std::vector<Eigen::Vector3d> knots_;  // A.k.a. "control points"

   private:
    uint64_t t0_ns_;
    uint64_t delta_t_ns_;
};

TEST(r3Spline, TestXXX) { EXPECT_EQ(1, 2); }
