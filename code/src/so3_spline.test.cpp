#include <gtest/gtest.h>

#include "constants.hpp"
#include "types.hpp"
#include "utilities.hpp"

namespace reprojection_calibration::spline {

// TODO(Jack): There is a non-trivial amount of copying and pasting between the r3 and s03 spline classes. Keep our eyes
// peeled for cost effective and well abstracted optimizations!
class So3Spline {
   public:
    So3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns) : t0_ns_{t0_ns}, delta_t_ns_{delta_t_ns} {}

    std::optional<Eigen::Matrix3d> Evaluate(uint64_t const t_ns,
                                            DerivativeOrder const derivative = DerivativeOrder::Null) const {
        (void)derivative;

        auto const [u_i, i]{NormalizedSegmentTime(t0_ns_, t_ns, delta_t_ns_)};

        if (std::size(knots_) < static_cast<size_t>(i + constants::k)) {
            return std::nullopt;
        }

        return Eigen::Matrix3d::Identity();
    }

    static VectorK CalculateU(double const u_i, DerivativeOrder const derivative = DerivativeOrder::Null);

    // NOTE(Jack): It would feel more natural to store the so3 vectors here but the math required in the evaluate
    // function happens more in the SO3 space so it makes more sense to have the knots be in that format - it is also
    // what people would expect to get returned from the Evaluate() function, so we are consistent.
    // TODO(Jack): When adding a knot should we check that it is a rotation matrix?
    std::vector<Eigen::Matrix3d> knots_;

   private:
    uint64_t t0_ns_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection_calibration::spline

using namespace reprojection_calibration::spline;

TEST(So3Spline, TestSo3SplineInvalidEvaluateConditions) {
    // Completely empty spline
    So3Spline so3_spline{100, 5};
    EXPECT_EQ(so3_spline.Evaluate(115), std::nullopt);

    // Add four knots which means we can ask for evaluations within the one time segment at the very start of the spline
    for (int i{0}; i < constants::k; ++i) {
        so3_spline.knots_.push_back(Eigen::Matrix3d::Identity());
    }

    EXPECT_NE(so3_spline.Evaluate(100), std::nullopt);  // Inside first time segment - valid
    EXPECT_EQ(so3_spline.Evaluate(105), std::nullopt);  // Outside first time segment - invalid

    // Add one more knot to see that we can now do a valid evaluation in the second time segment
    so3_spline.knots_.push_back(Eigen::Matrix3d::Identity());
    EXPECT_NE(so3_spline.Evaluate(105), std::nullopt);
}