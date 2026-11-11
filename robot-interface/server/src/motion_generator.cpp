#include "csc376_franky/motion_generator.hpp"

#include <algorithm>
#include <cmath>

namespace csc376_franky
{

namespace
{

constexpr double kDeltaQMotionFinished = 1e-6;
constexpr Vec7 kDqMax = {2.0, 2.0, 2.0, 2.0, 2.5, 2.5, 2.5};
constexpr double kDdqMaxStart = 5.0;
constexpr double kDdqMaxGoal = 5.0;

double sign(double value)
{
    return static_cast<double>((value > 0.0) - (value < 0.0));
}

} // namespace

JointMotionGenerator::JointMotionGenerator(const Vec7& q_start, const Vec7& q_goal, double speed_factor)
    : q_start_(q_start)
{
    if (!(speed_factor > 0.0 && speed_factor <= 1.0))
        throw std::invalid_argument("speed_factor must be in (0, 1]");
    const double ddq_start = kDdqMaxStart * speed_factor;
    const double ddq_goal = kDdqMaxGoal * speed_factor;

    // calculateSynchronizedValues(): the slowest joint sets the duration, the
    // others lower their top speed to arrive with it.
    Vec7 t_f{};
    for (int i = 0; i < kDof; ++i)
    {
        delta_q_[i] = q_goal[i] - q_start[i];
        if (std::abs(delta_q_[i]) <= kDeltaQMotionFinished)
            continue;
        const double dq_max = kDqMax[i] * speed_factor;
        double dq_max_reach = dq_max;
        if (std::abs(delta_q_[i]) < 0.75 * (dq_max * dq_max / ddq_start) + 0.75 * (dq_max * dq_max / ddq_goal))
            dq_max_reach =
                std::sqrt(4.0 / 3.0 * std::abs(delta_q_[i]) * (ddq_start * ddq_goal) / (ddq_start + ddq_goal));
        const double t_1 = 1.5 * dq_max_reach / ddq_start;
        const double delta_t_2 = 1.5 * dq_max_reach / ddq_goal;
        t_f[i] = t_1 / 2.0 + delta_t_2 / 2.0 + std::abs(delta_q_[i]) / dq_max_reach;
    }
    const double max_t_f = *std::max_element(t_f.begin(), t_f.end());
    for (int i = 0; i < kDof; ++i)
    {
        if (std::abs(delta_q_[i]) <= kDeltaQMotionFinished)
            continue;
        const double a = 1.5 / 2.0 * (ddq_goal + ddq_start);
        const double b = -1.0 * max_t_f * ddq_goal * ddq_start;
        const double c = std::abs(delta_q_[i]) * ddq_goal * ddq_start;
        const double delta = std::max(0.0, b * b - 4.0 * a * c);
        dq_max_sync_[i] = (-1.0 * b - std::sqrt(delta)) / (2.0 * a);
        t_1_sync_[i] = 1.5 * dq_max_sync_[i] / ddq_start;
        const double delta_t_2_sync = 1.5 * dq_max_sync_[i] / ddq_goal;
        t_f_sync_[i] = t_1_sync_[i] / 2.0 + delta_t_2_sync / 2.0 + std::abs(delta_q_[i] / dq_max_sync_[i]);
        t_2_sync_[i] = t_f_sync_[i] - delta_t_2_sync;
        q_1_[i] = dq_max_sync_[i] * sign(delta_q_[i]) * (0.5 * t_1_sync_[i]);
    }
    duration_ = *std::max_element(t_f_sync_.begin(), t_f_sync_.end());
}

// calculateDesiredValues()
bool JointMotionGenerator::positions(double t, Vec7& q_d) const
{
    bool finished = true;
    for (int i = 0; i < kDof; ++i)
    {
        const double s = sign(delta_q_[i]);
        const double t_d = t_2_sync_[i] - t_1_sync_[i];
        const double delta_t_2_sync = t_f_sync_[i] - t_2_sync_[i];
        double delta_q_d = 0.0;
        if (std::abs(delta_q_[i]) < kDeltaQMotionFinished)
        {
            delta_q_d = 0.0;
        }
        else if (t < t_1_sync_[i])
        {
            delta_q_d =
                -1.0 / std::pow(t_1_sync_[i], 3.0) * dq_max_sync_[i] * s * (0.5 * t - t_1_sync_[i]) * std::pow(t, 3.0);
            finished = false;
        }
        else if (t < t_2_sync_[i])
        {
            delta_q_d = q_1_[i] + (t - t_1_sync_[i]) * dq_max_sync_[i] * s;
            finished = false;
        }
        else if (t < t_f_sync_[i])
        {
            delta_q_d = delta_q_[i] +
                        0.5 *
                            (1.0 / std::pow(delta_t_2_sync, 3.0) * (t - t_1_sync_[i] - 2.0 * delta_t_2_sync - t_d) *
                                 std::pow(t - t_1_sync_[i] - t_d, 3.0) +
                             (2.0 * t - 2.0 * t_1_sync_[i] - delta_t_2_sync - 2.0 * t_d)) *
                            dq_max_sync_[i] * s;
            finished = false;
        }
        else
        {
            delta_q_d = delta_q_[i];
        }
        q_d[i] = q_start_[i] + delta_q_d;
    }
    return finished;
}

} // namespace csc376_franky
