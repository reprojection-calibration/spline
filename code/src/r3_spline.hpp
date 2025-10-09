#include <Eigen/Dense>
#include <optional>

namespace reprojection_calibration::spline {

namespace constants {

// Instead of templating everything like mad we will use this global to parameterize the order of the spline.
inline constexpr int k{4};  // Spline order - note spline "degree" is k-1, so when k=4 it is a cubic spline!
inline constexpr int d{3};  // State dimension for r3 spline

}  // namespace constants

using MatrixDK = Eigen::Matrix<double, constants::d, constants::k>;
using MatrixKK = Eigen::Matrix<double, constants::k, constants::k>;
using VectorD = Eigen::Vector<double, constants::d>;
using VectorK = Eigen::Vector<double, constants::k>;

// NOTE(Jack): It is a unfiform spline which presupposes that all added knots correspond to specific evenly spaced
// times.
class r3Spline {
   public:
    r3Spline(uint64_t const t0_ns, uint64_t const delta_t_ns);

    // TODO(Jack): use enum and static cast instead of int
    std::optional<VectorD> Evaluate(uint64_t const t_ns, int const derivative) const;

    // TODO(Jack): Let us consider what benefit we would get from making this private at some later point
    std::vector<VectorD> knots_;  // A.k.a. "control points"

   private:
    uint64_t t0_ns_;
    uint64_t delta_t_ns_;
};

}  // namespace reprojection_calibration::spline
