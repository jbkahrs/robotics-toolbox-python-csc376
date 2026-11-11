#include "csc376_franky/trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace csc376_franky
{

namespace
{

// How long every joint must stay below settle_velocity before the motion ends [s].
constexpr double kStillTime = 0.05;

std::string num(double value)
{
    std::ostringstream out;
    out << std::setprecision(4) << value;
    return out.str();
}

std::string joint(int j)
{
    return "q[" + std::to_string(j) + "]";
}

ValidationResult fail(ErrorCodes code, std::string message)
{
    return ValidationResult{code, std::move(message)};
}

} // namespace

ValidationResult validateTrajectory(
    const std::vector<Vec7>& points, double dt, const Vec7* q_current, const Limits& limits)
{
    if (!std::isfinite(dt))
        return fail(ErrorCodes::NonFiniteValue, "dt must be a finite number");
    if (dt <= 0.0)
        return fail(ErrorCodes::DtIsNegative, "dt must be larger than 0");
    if (dt < limits.min_dt)
        return fail(ErrorCodes::DtIsSmallerThanAllowed,
                    "dt of " + num(dt) + " s is below the minimum of " + num(limits.min_dt) + " s");
    if (dt > limits.max_dt)
        return fail(ErrorCodes::DtIsLargerThanAllowed,
                    "dt of " + num(dt) + " s is above the maximum of " + num(limits.max_dt) + " s");
    if (points.size() < 2)
        return fail(ErrorCodes::TrajectorySizeLessThanTwo, "Trajectory must have at least 2 points");
    if (points.size() > limits.max_points)
        return fail(ErrorCodes::TrajectoryTooLong,
                    "Trajectory has " + std::to_string(points.size()) + " points; the maximum is " +
                        std::to_string(limits.max_points));
    const double duration = dt * static_cast<double>(points.size() - 1);
    if (duration > limits.max_duration)
        return fail(ErrorCodes::TrajectoryTooLong,
                    "Trajectory lasts " + num(duration) + " s; the maximum is " + num(limits.max_duration) + " s");

    for (std::size_t i = 0; i < points.size(); ++i)
    {
        for (int j = 0; j < kDof; ++j)
        {
            if (!std::isfinite(points[i][j]))
                return fail(ErrorCodes::NonFiniteValue,
                            "Point " + std::to_string(i) + ", " + joint(j) + " is not a finite number");
        }
    }

    if (q_current != nullptr)
    {
        for (int j = 0; j < kDof; ++j)
        {
            const double gap = std::abs(points.front()[j] - (*q_current)[j]);
            if (gap > limits.start_tolerance)
                return fail(ErrorCodes::JointTrajectoryDoesNotStartAtCurrentPosition,
                            "The trajectory starts " + num(gap) + " rad away from the robot's current position on " +
                                joint(j) + " (tolerance " + num(limits.start_tolerance) +
                                " rad). Plan again from the robot's current joint positions.");
        }
    }

    for (std::size_t i = 0; i < points.size(); ++i)
    {
        for (int j = 0; j < kDof; ++j)
        {
            const double low = kJointPositionMin[j] + limits.joint_limit_margin;
            const double high = kJointPositionMax[j] - limits.joint_limit_margin;
            if (points[i][j] < low || points[i][j] > high)
                return fail(ErrorCodes::JointLimitsExceeded,
                            "Point " + std::to_string(i) + ": " + joint(j) + " = " + num(points[i][j]) +
                                " rad is outside the allowed range [" + num(low) + ", " + num(high) + "]");
        }
    }

    for (std::size_t i = 0; i + 1 < points.size(); ++i)
    {
        for (int j = 0; j < kDof; ++j)
        {
            const double step = std::abs(points[i + 1][j] - points[i][j]);
            const std::string where = " between points " + std::to_string(i) + " and " + std::to_string(i + 1);
            if (step > limits.max_joint_step)
                return fail(ErrorCodes::JointDistancesAboveMaxJointDifference,
                            joint(j) + " changes by " + num(step) + " rad" + where + "; the maximum is " +
                                num(limits.max_joint_step) +
                                " rad. Use more points (a smaller dt), and check for an inverse kinematics "
                                "solution flipping to another configuration.");
            const double velocity = step / dt;
            const double velocity_max = limits.velocity_scale * kJointVelocityMax[j];
            if (velocity > velocity_max)
                return fail(ErrorCodes::JointVelocitiesAboveMaximumAllowed,
                            joint(j) + " would move at " + num(velocity) + " rad/s" + where + "; the maximum is " +
                                num(velocity_max) + " rad/s. Slow the trajectory down.");
        }
    }

    // The follower feeds each segment's velocity forward, so a change of
    // velocity at a point is a step in the commanded torque. The first and the
    // last segment are not compared with rest: a constant-velocity trajectory
    // (np.linspace) has always been accepted.
    for (std::size_t i = 1; i + 1 < points.size(); ++i)
    {
        for (int j = 0; j < kDof; ++j)
        {
            const double before = points[i][j] - points[i - 1][j];
            const double after = points[i + 1][j] - points[i][j];
            const double acceleration = std::abs(after - before) / (dt * dt);
            const double acceleration_max = limits.acceleration_scale * kJointAccelerationMax[j];
            if (acceleration > acceleration_max)
                return fail(ErrorCodes::JointAccelerationsAboveMaximumAllowed,
                            joint(j) + " would accelerate at " + num(acceleration) + " rad/s^2 at point " +
                                std::to_string(i) + "; the maximum is " + num(acceleration_max) +
                                " rad/s^2. Change the velocity over more points instead of at one.");
        }
    }
    return {};
}

TrajectoryFollower::TrajectoryFollower(std::vector<Vec7> points, double dt, const Limits& limits)
    : points_(std::move(points)), dt_(dt), duration_(0.0), limits_(limits)
{
    if (points_.size() < 2 || !(dt_ > 0.0))
        throw std::invalid_argument("TrajectoryFollower needs at least 2 points and dt > 0");
    duration_ = dt_ * static_cast<double>(points_.size() - 1);
    q_d_ = points_.front();
}

void TrajectoryFollower::step(double period, const Vec7& q, const Vec7& dq, bool stop_requested)
{
    if (phase_ == Phase::Done)
        return;
    if (!started_)
    {
        started_ = true;
        for (int j = 0; j < kDof; ++j)
            start_offset_[j] = q[j] - points_.front()[j];
    }
    elapsed_ += period;

    if (stop_requested && phase_ == Phase::Running)
    {
        phase_ = Phase::Stopping;
        interrupted_ = true;
    }
    if (phase_ == Phase::Stopping)
        rate_ = limits_.stop_time > 0.0 ? std::max(0.0, rate_ - period / limits_.stop_time) : 0.0;
    if (phase_ == Phase::Running || phase_ == Phase::Stopping)
    {
        t_ = std::min(duration_, t_ + rate_ * period);
        // A stop that came too late to end the playback short of the last
        // point did not interrupt anything.
        if (t_ >= duration_)
            interrupted_ = false;
        if (t_ >= duration_ || rate_ <= 0.0)
            phase_ = Phase::Settling;
    }

    const std::size_t last = points_.size() - 1;
    std::size_t i = static_cast<std::size_t>(t_ / dt_);
    if (i >= last)
        i = last - 1;
    const double s = std::clamp((t_ - dt_ * static_cast<double>(i)) / dt_, 0.0, 1.0);
    const double feed_forward = phase_ == Phase::Settling ? 0.0 : rate_;

    // Weight of the start offset: 1 at the start, 0 after start_blend_time,
    // with zero slope at both ends (smoothstep).
    double w = 0.0;
    double dw = 0.0;
    if (limits_.start_blend_time > 0.0 && elapsed_ < limits_.start_blend_time)
    {
        const double u = elapsed_ / limits_.start_blend_time;
        w = 1.0 - u * u * (3.0 - 2.0 * u);
        dw = -6.0 * u * (1.0 - u) / limits_.start_blend_time;
    }

    for (int j = 0; j < kDof; ++j)
    {
        const double a = points_[i][j];
        const double b = points_[i + 1][j];
        q_d_[j] = a + s * (b - a) + w * start_offset_[j];
        dq_d_[j] = feed_forward * (b - a) / dt_ + dw * start_offset_[j];
    }

    if (phase_ == Phase::Settling)
    {
        settle_elapsed_ += period;
        double fastest = 0.0;
        for (int j = 0; j < kDof; ++j)
            fastest = std::max(fastest, std::abs(dq[j]));
        still_time_ = fastest < limits_.settle_velocity ? still_time_ + period : 0.0;
        const bool blended = elapsed_ >= limits_.start_blend_time;
        if ((blended && still_time_ >= kStillTime) || settle_elapsed_ >= limits_.settle_timeout)
            phase_ = Phase::Done;
    }
}

std::size_t TrajectoryFollower::index() const
{
    const std::size_t i = static_cast<std::size_t>(t_ / dt_ + 1e-9);
    return std::min(i, points_.size() - 1);
}

} // namespace csc376_franky
