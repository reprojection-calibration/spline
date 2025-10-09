#include <gtest/gtest.h>

#include "utilities.hpp"

using namespace reprojection_calibration::spline;

namespace reprojection_calibration::spline::constants {

// Instead of templating everything like mad we will use this global to parameterize the order of the spline.
inline constexpr int k{4};  // Spline order - note spline "degree" is k-1, so when k=4 it is a cubic spline!
inline constexpr int d{3};  // State dimension for r3 spline

}  // namespace reprojection_calibration::spline::constants

using MatrixDK = Eigen::Matrix<double, constants::d, constants::k>;
using MatrixKK = Eigen::Matrix<double, constants::k, constants::k>;
using VectorK = Eigen::Vector<double, constants::k>;

// NOTE(Jack): It is a unfiform spline which presupposes that all added knots correspond to specific evenly spaced
// times.
class r3Spline {
   public:
    r3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns) : t0_ns_{t0_ns}, delta_t_ns_{delta_t_ns} {}

    std::optional<Eigen::Vector3d> Evaluate(uint64_t const t_ns) const {
        auto const [u_i, i]{NormalizedSegmentTime(t0_ns_, t_ns, delta_t_ns_)};

        // From reference [1] - "At time t in [t_i, t_i+1) the value of p(t) only depends on the control points p_i,
        // p_i+1, ..., p_i+k-1" - See the start of the second paragraph in section 4.2 Matrix Representation.
        if (std::size(knots_) < static_cast<size_t>(i + constants::k)) {
            return std::nullopt;
        }

        MatrixDK const P{Eigen::Map<const MatrixDK>(knots_[i].data(), constants::d, constants::k)};
        static MatrixKK const M{BlendingMatrix(constants::k)};
        VectorK const u{TimePolynomial(constants::k, u_i)};

        return P * M * u;
    }

    // TODO(Jack): Let us consider what benefit we would get from making this private at some later point
    std::vector<Eigen::Vector3d> knots_;  // A.k.a. "control points"

   private:
    uint64_t t0_ns_;
    uint64_t delta_t_ns_;
};

TEST(r3Spline, Testr3SplineInvalidEvaluateConditions) {
    // Completely empty spline
    r3Spline r3_spline{100, 5};
    EXPECT_EQ(r3_spline.Evaluate(115), std::nullopt);

    // Add four knots which means we can ask for evaluations within the one time segment at the very start of the spline
    for (int i{0}; i < constants::k; ++i) {
        r3_spline.knots_.push_back(Eigen::Vector3d::Zero());
    }

    EXPECT_NE(r3_spline.Evaluate(100), std::nullopt);  // Inside first time segment - valid
    EXPECT_EQ(r3_spline.Evaluate(105), std::nullopt);  // Outside first time segment - invalid

    // Add one more knot to see that we can now do a valid evaluation in the second time segment
    r3_spline.knots_.push_back(Eigen::Vector3d::Zero());
    EXPECT_NE(r3_spline.Evaluate(105), std::nullopt);
}
