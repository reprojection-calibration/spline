#include <gtest/gtest.h>

#include "constants.hpp"
#include "lie.hpp"
#include "r3_spline.hpp"  // REMOVE AND USE COMMON IMPLEMENTATIONS
#include "types.hpp"
#include "utilities.hpp"

// Testing Util - COPY AND PASTED
bool IsRotation(Eigen::Matrix3d const& R) {
    Eigen::Matrix3d const RRT{R * R.transpose()};  // For rotations R^T = R^-1
    bool const is_orthogonal{(RRT - Eigen::Matrix3d::Identity()).norm() < 1e-10};

    double const D{R.determinant()};
    bool const is_proper{(D - 1) < std::numeric_limits<double>::epsilon()};  // Determinant is positive one

    return is_orthogonal and is_proper;
}

namespace reprojection_calibration::spline {

// TODO(Jack): There is a non-trivial amount of copying and pasting between the r3 and s03 spline classes. Keep our eyes
// peeled for cost effective and well abstracted optimizations!
class So3Spline {
   public:
    So3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns) : t0_ns_{t0_ns}, delta_t_ns_{delta_t_ns} {}

    std::optional<Eigen::Matrix3d> Evaluate(uint64_t const t_ns,
                                            DerivativeOrder const derivative = DerivativeOrder::Null) const {
        auto const [u_i, i]{NormalizedSegmentTime(t0_ns_, t_ns, delta_t_ns_)};

        if (std::size(knots_) < static_cast<size_t>(i + constants::k)) {
            return std::nullopt;
        }

        static MatrixKK const M{CumulativeBlendingMatrix(constants::k)};  // Static means it only evaluates once :)
        VectorK const u{r3Spline::CalculateU(u_i, derivative)}; // Use common one!

        VectorK const weights{M * u};  // TODO NAME

        Eigen::Matrix3d result{knots_[i]};
        for (int j{0}; j < (constants::k - 1); ++j) {
            Eigen::Matrix3d const& p0{knots_[i + j]};
            Eigen::Matrix3d const& p1{knots_[i + j + 1]};

            Eigen::Matrix3d const r01{p0.inverse() * p1};
            Eigen::Vector3d const delta{Log(r01)};
            Eigen::Vector3d const kdelta = delta * weights[j + 1];

            result *= Exp(kdelta);
        }

        return result;
    }

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

TEST(So3Spline, TestSo3SplineEvaluate) {
    So3Spline so3_spline{100, 5};
    so3_spline.knots_.push_back(Exp(Eigen::Vector3d::Zero()));

    for (int i{1}; i < constants::k; ++i) {
        so3_spline.knots_.push_back(so3_spline.knots_.back() *
                                    Exp((static_cast<double>(i) / 10) * Eigen::Vector3d::Ones()));
    }

    // Heuristic test as we have no theoretical testing strategy at this time.
    // THIS IS THE delta_t - use variable here
    for (int i{0}; i < 5; ++i) {
        auto const p_i{so3_spline.Evaluate(100 + i)};
        ASSERT_TRUE(p_i.has_value());
        EXPECT_TRUE(IsRotation(p_i.value()));
    }

    auto const p_0{so3_spline.Evaluate(100)};
    EXPECT_FLOAT_EQ(p_0.value().diagonal().sum(),
                    2.9593055);  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
                                 // that we can detect changes to the implementation quickly (hopefully. )
}