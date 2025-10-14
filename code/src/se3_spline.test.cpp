#include <gtest/gtest.h>

#include "lie.hpp"
#include "r3_spline.hpp"
#include "so3_spline.hpp"
#include "utilities_testing.hpp"

namespace reprojection_calibration::spline {

class Se3Spline {
   public:
    Se3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns)
        : r3_spline_{t0_ns, delta_t_ns}, so3_spline_{t0_ns, delta_t_ns} {}

    void AddKnot(Eigen::Isometry3d const knot) {
        r3_spline_.knots_.push_back(knot.translation());
        so3_spline_.knots_.push_back(knot.linear());
    }

    std::optional<Eigen::Isometry3d> Evaluate(uint64_t const t_ns) const {
        // TODO(Jack): This is in essence repeating logic that we already have implemented elsewhere, is there anything
        // we can do to streamline this?
        auto const position{r3_spline_.Evaluate(t_ns)};
        auto const rotation{so3_spline_.Evaluate(t_ns)};
        if (not(position.has_value() and rotation.has_value())) {
            return std::nullopt;
        }

        Eigen::Isometry3d result{Eigen::Isometry3d::Identity()};
        result.rotate(rotation.value());
        result.translation() = position.value();

        return result;
    }

   private:
    r3Spline r3_spline_;
    So3Spline so3_spline_;
};

}  // namespace reprojection_calibration::spline

using namespace reprojection_calibration::spline;

TEST(Se3Spline, TestSe3SplineInvalidEvaluateConditions) {
    Se3Spline se3_spline{100, 5};
    EXPECT_EQ(se3_spline.Evaluate(115), std::nullopt);

    for (int i{0}; i < constants::k; ++i) {
        se3_spline.AddKnot(Eigen::Isometry3d::Identity());
    }

    EXPECT_NE(se3_spline.Evaluate(100), std::nullopt);
    EXPECT_EQ(se3_spline.Evaluate(105), std::nullopt);

    se3_spline.AddKnot(Eigen::Isometry3d::Identity());
    EXPECT_NE(se3_spline.Evaluate(105), std::nullopt);
}

TEST(Se3Spline, TestSe3SplineEvaluate) {
    uint64_t const delta_t_ns{5};
    Se3Spline se3_spline{100, delta_t_ns};

    Eigen::Isometry3d knot_i{Eigen::Isometry3d::Identity()};
    se3_spline.AddKnot(knot_i);

    for (int i{1}; i < constants::k; ++i) {
        Eigen::Isometry3d delta{Eigen::Isometry3d::Identity()};
        delta.rotate(Exp((static_cast<double>(i) / 10) * Eigen::Vector3d::Ones()));
        delta.translation() = i * VectorD::Ones();  // Using defined constant types

        knot_i = delta * knot_i;

        se3_spline.AddKnot(knot_i);
    }

    // Heuristic test as we have no theoretical testing strategy at this time.
    for (int i{0}; i < static_cast<int>(delta_t_ns); ++i) {
        auto const p_i{se3_spline.Evaluate(100 + i)};
        ASSERT_TRUE(p_i.has_value());
        EXPECT_TRUE(IsRotation(p_i->rotation().matrix()));
    }

    auto const p_0{se3_spline.Evaluate(100)};
    EXPECT_FLOAT_EQ(p_0->matrix().diagonal().sum(),
                    3.9593055);  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
    // that we can detect changes to the implementation quickly (hopefully. )
}