#pragma once

#include "csc376_franky/common.hpp"

namespace csc376_franky
{

// libfranka's example MotionGenerator (examples/examples_common.cpp, Apache-2.0),
// without Eigen: a joint motion from q_start to q_goal, at rest at both ends,
// all joints synchronized to arrive together. speed_factor scales libfranka's
// limits of 2.0 (2.5 for joints 5 to 7) rad/s and 5 rad/s^2.
class JointMotionGenerator
{
public:
    JointMotionGenerator(const Vec7& q_start, const Vec7& q_goal, double speed_factor);

    // The joint positions t seconds after the start; true once every joint has arrived.
    bool positions(double t, Vec7& q_d) const;
    double duration() const { return duration_; }

private:
    Vec7 q_start_{};
    Vec7 delta_q_{};
    Vec7 dq_max_sync_{};
    Vec7 t_1_sync_{};
    Vec7 t_2_sync_{};
    Vec7 t_f_sync_{};
    Vec7 q_1_{};
    double duration_ = 0.0;
};

} // namespace csc376_franky
