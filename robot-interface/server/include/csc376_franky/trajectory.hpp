#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "csc376_franky/common.hpp"

namespace csc376_franky
{

struct ValidationResult
{
    ErrorCodes code = ErrorCodes::Success;
    std::string message;

    bool ok() const { return code == ErrorCodes::Success; }
};

// Checks a joint trajectory (points dt seconds apart) against the limits.
// q_current is the robot's measured position; nullptr skips the start check.
ValidationResult validateTrajectory(
    const std::vector<Vec7>& points, double dt, const Vec7* q_current, const Limits& limits);

// Plays a validated trajectory back at the control rate.
//
// Between points the command is interpolated linearly, with the segment's
// velocity as feed-forward, so the robot only ever follows the straight
// segments between validated points. Whatever gap there is between the
// measured start and the first point is blended out over start_blend_time.
// After stop the playback rate ramps down to zero over stop_time, so the
// robot comes to rest on the planned path. At the end the command holds the
// last position until the robot has settled.
//
// step() does not allocate, so it is safe in the 1 kHz control callback.
class TrajectoryFollower
{
public:
    enum class Phase
    {
        Running,
        Stopping,
        Settling,
        Done,
    };

    TrajectoryFollower(std::vector<Vec7> points, double dt, const Limits& limits);

    // Advance by period seconds; q and dq are the measured joint state.
    void step(double period, const Vec7& q, const Vec7& dq, bool stop_requested);

    const Vec7& qDesired() const { return q_d_; }
    const Vec7& dqDesired() const { return dq_d_; }
    const Vec7& firstPoint() const { return points_.front(); }
    Phase phase() const { return phase_; }
    // True if stop ended the playback before the last point.
    bool interrupted() const { return interrupted_; }
    // Index of the last point the playback has passed.
    std::size_t index() const;
    std::size_t length() const { return points_.size(); }
    double time() const { return t_; }
    double duration() const { return duration_; }

private:
    std::vector<Vec7> points_;
    double dt_;
    double duration_;
    Limits limits_;

    bool started_ = false;
    Vec7 start_offset_{};
    double elapsed_ = 0.0;
    double t_ = 0.0;
    double rate_ = 1.0;
    double settle_elapsed_ = 0.0;
    double still_time_ = 0.0;
    bool interrupted_ = false;
    Phase phase_ = Phase::Running;
    Vec7 q_d_{};
    Vec7 dq_d_{};
};

} // namespace csc376_franky
