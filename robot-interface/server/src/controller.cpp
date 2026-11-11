#include "csc376_franky/controller.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <optional>
#include <sstream>

#include "csc376_franky/motion_generator.hpp"

namespace csc376_franky
{

namespace
{

constexpr auto kReconnectInterval = std::chrono::seconds(2);
constexpr std::size_t kMaxResults = 32;

std::string num(double value)
{
    std::ostringstream out;
    out << std::setprecision(4) << value;
    return out.str();
}

// Why the robot cannot start a motion in this mode, or nullptr if it can.
const char* notReadyReason(int mode)
{
    switch (static_cast<RobotMode>(mode))
    {
    case RobotMode::Idle: return nullptr;
    case RobotMode::Reflex:
        return "The robot stopped after a reflex (for example a collision) and needs an error recovery: call "
               "recover().";
    case RobotMode::UserStopped: return "The user stop is pressed. Release it, then try again.";
    case RobotMode::Guiding: return "The robot is being guided by hand. Let go of the guiding buttons, then try again.";
    case RobotMode::AutomaticErrorRecovery: return "The robot is recovering from an error; try again in a moment.";
    case RobotMode::Move: return "The robot is moving.";
    case RobotMode::Other: break;
    }
    return "The robot is not ready to move: open the brakes and activate FCI in Desk.";
}

} // namespace

Controller::Controller(std::unique_ptr<Backend> backend, const Limits& limits)
    : backend_(std::move(backend)), limits_(limits)
{
    thread_ = std::thread(&Controller::run, this);
}

Controller::~Controller()
{
    shutdown();
}

void Controller::shutdown()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
        finishPendingLocked(ErrorCodes::Interrupted, "The robot server is shutting down");
        stop_requested_ = true;
        wake_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable())
        thread_.join();
}

void Controller::setConnectionWanted(bool wanted)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (want_connection_ == wanted)
            return;
        want_connection_ = wanted;
        if (!wanted)
        {
            finishPendingLocked(ErrorCodes::RobotNotConnected, "The robot server is disconnecting from the robot");
            stop_requested_ = true;
        }
        wake_ = true;
    }
    cv_.notify_all();
}

bool Controller::busyLocked() const
{
    return status_ == Status::Executing || status_ == Status::Recovering || pending_ != Pending::None;
}

ErrorCodes Controller::notConnectedLocked(std::string& message) const
{
    message = "The miniPC is not connected to the robot";
    if (!connection_error_.empty())
        message += ": " + connection_error_;
    else if (status_ == Status::Connecting || status_ == Status::Idle)
        message += " yet; try again in a moment";
    return ErrorCodes::RobotNotConnected;
}

ValidationResult Controller::validate(const std::vector<Vec7>& points, double dt) const
{
    Measured state;
    bool has_state = false;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        has_state = has_state_;
        state = state_;
    }
    return validateTrajectory(points, dt, has_state ? &state.q : nullptr, limits_);
}

ErrorCodes Controller::readyLocked(std::string& message, Measured& state) const
{
    if (busyLocked())
    {
        message = status_ == Status::Recovering ? "The robot is recovering from an error"
                                                : "The robot is already executing a trajectory";
        return ErrorCodes::RobotBusy;
    }
    if (shutdown_ || !want_connection_ || status_ != Status::Idle)
        return notConnectedLocked(message);
    {
        std::lock_guard<std::mutex> state_lock(state_mutex_);
        if (!has_state_)
            return notConnectedLocked(message);
        state = state_;
    }
    if (const char* reason = notReadyReason(state.robot_mode))
    {
        message = reason;
        return ErrorCodes::RobotNotReady;
    }
    return ErrorCodes::Success;
}

Controller::Submission Controller::execute(std::vector<Vec7> points, double dt)
{
    std::lock_guard<std::mutex> lock(mutex_);
    Submission submission;
    Measured state;
    submission.code = readyLocked(submission.message, state);
    if (submission.code != ErrorCodes::Success)
        return submission;
    const ValidationResult validation = validateTrajectory(points, dt, &state.q, limits_);
    if (!validation.ok())
    {
        submission.code = validation.code;
        submission.message = validation.message;
        return submission;
    }
    pending_ = Pending::Execute;
    pending_id_ = next_id_++;
    pending_points_ = std::move(points);
    pending_dt_ = dt;
    submission.id = pending_id_;
    wake_ = true;
    cv_.notify_all();
    return submission;
}

Controller::Submission Controller::moveToJointPositions(const Vec7& q_goal, double speed_factor)
{
    std::lock_guard<std::mutex> lock(mutex_);
    Submission submission;
    if (!(speed_factor > 0.0 && speed_factor <= limits_.velocity_scale))
    {
        submission.code = ErrorCodes::JointVelocitiesAboveMaximumAllowed;
        submission.message = "speed_factor must be larger than 0 and at most " + num(limits_.velocity_scale);
        return submission;
    }
    for (int j = 0; j < kDof; ++j)
    {
        const double low = kJointPositionMin[j] + limits_.joint_limit_margin;
        const double high = kJointPositionMax[j] - limits_.joint_limit_margin;
        if (!std::isfinite(q_goal[j]))
        {
            submission.code = ErrorCodes::NonFiniteValue;
            submission.message = "q_goal[" + std::to_string(j) + "] is not a finite number";
            return submission;
        }
        if (q_goal[j] < low || q_goal[j] > high)
        {
            submission.code = ErrorCodes::JointLimitsExceeded;
            submission.message = "q_goal[" + std::to_string(j) + "] = " + num(q_goal[j]) +
                                 " rad is outside the allowed range [" + num(low) + ", " + num(high) + "]";
            return submission;
        }
    }
    Measured state;
    submission.code = readyLocked(submission.message, state);
    if (submission.code != ErrorCodes::Success)
        return submission;
    pending_ = Pending::JointMotion;
    pending_id_ = next_id_++;
    pending_goal_ = q_goal;
    pending_speed_factor_ = speed_factor;
    submission.id = pending_id_;
    wake_ = true;
    cv_.notify_all();
    return submission;
}

Controller::Submission Controller::recover()
{
    std::lock_guard<std::mutex> lock(mutex_);
    Submission submission;
    if (busyLocked())
    {
        submission.code = ErrorCodes::RobotBusy;
        submission.message = "The robot is busy; recover() only runs while it is idle";
        return submission;
    }
    if (shutdown_ || !want_connection_ || status_ != Status::Idle)
    {
        submission.code = notConnectedLocked(submission.message);
        return submission;
    }
    pending_ = Pending::Recover;
    pending_id_ = next_id_++;
    submission.id = pending_id_;
    wake_ = true;
    cv_.notify_all();
    return submission;
}

void Controller::stop()
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (pending_ == Pending::Execute || pending_ == Pending::JointMotion)
        finishPendingLocked(ErrorCodes::Interrupted, "The trajectory was stopped before it started");
    if (status_ == Status::Executing)
        stop_requested_ = true;
}

Controller::Snapshot Controller::snapshot() const
{
    Snapshot s;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        s.status = status_;
        s.connected = status_ == Status::Idle || status_ == Status::Executing || status_ == Status::Recovering;
        s.connection_error = connection_error_;
        s.active_id = active_id_;
        s.results.assign(results_.begin(), results_.end());
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        s.has_state = has_state_;
        s.seq = seq_;
        s.state = state_;
        s.age = std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp_).count();
        if (s.active_id != 0)
        {
            s.index = index_;
            s.length = length_;
            s.time = time_;
            s.duration = duration_;
            s.stopping = stopping_;
            s.settling = settling_;
            s.q_d = q_d_;
            s.dq_d = dq_d_;
        }
    }
    if (s.connected)
        backend_->errors(s.current_errors, s.last_motion_errors);
    return s;
}

void Controller::run()
{
    std::unique_lock<std::mutex> lock(mutex_);
    while (!shutdown_)
    {
        if (!want_connection_)
        {
            if (backend_->isConnected())
            {
                lock.unlock();
                backend_->disconnect();
                clearState();
                lock.lock();
            }
            status_ = Status::Disconnected;
            connection_error_.clear();
            cv_.wait(lock, [this] { return shutdown_ || want_connection_; });
            continue;
        }

        if (!backend_->isConnected())
        {
            status_ = Status::Connecting;
            lock.unlock();
            clearState();
            std::string error;
            try
            {
                backend_->connect();
            }
            catch (const std::exception& e)
            {
                error = e.what();
                if (error.empty())
                    error = "unknown error";
            }
            lock.lock();
            if (!error.empty())
            {
                status_ = Status::Disconnected;
                connection_error_ = error;
                cv_.wait_for(lock, kReconnectInterval, [this] { return shutdown_ || !want_connection_; });
                continue;
            }
            connection_error_.clear();
        }

        if (pending_ == Pending::Execute)
        {
            const std::uint64_t id = pending_id_;
            std::vector<Vec7> points = std::move(pending_points_);
            pending_points_.clear();
            const double dt = pending_dt_;
            pending_ = Pending::None;
            active_id_ = id;
            status_ = Status::Executing;
            stop_requested_ = false;
            lock.unlock();
            runTrajectory(id, std::move(points), dt);
            lock.lock();
            active_id_ = 0;
            if (status_ == Status::Executing)
                status_ = Status::Idle;
            continue;
        }

        if (pending_ == Pending::JointMotion)
        {
            const std::uint64_t id = pending_id_;
            const Vec7 q_goal = pending_goal_;
            const double speed_factor = pending_speed_factor_;
            pending_ = Pending::None;
            active_id_ = id;
            status_ = Status::Executing;
            stop_requested_ = false;
            lock.unlock();
            runJointMotion(id, q_goal, speed_factor);
            lock.lock();
            active_id_ = 0;
            if (status_ == Status::Executing)
                status_ = Status::Idle;
            continue;
        }

        if (pending_ == Pending::Recover)
        {
            const std::uint64_t id = pending_id_;
            pending_ = Pending::None;
            status_ = Status::Recovering;
            lock.unlock();
            runRecovery(id);
            lock.lock();
            if (status_ == Status::Recovering)
                status_ = Status::Idle;
            continue;
        }

        status_ = Status::Idle;
        wake_ = false;
        lock.unlock();
        std::string error;
        bool lost = false;
        try
        {
            backend_->read([this](const Measured& measured) {
                publish(measured, nullptr);
                // The robot is being read again, so the last read's error is over.
                if (read_failed_)
                {
                    std::unique_lock<std::mutex> error_lock(mutex_, std::try_to_lock);
                    if (error_lock.owns_lock())
                    {
                        connection_error_.clear();
                        read_failed_ = false;
                    }
                }
                return !wake_.load();
            });
        }
        catch (const BackendNetworkError& e)
        {
            lost = true;
            error = e.what();
        }
        catch (const std::exception& e)
        {
            error = e.what();
        }
        if (lost)
            connectionLost(error);
        lock.lock();
        if (lost)
            read_failed_ = false;
        if (!lost && !error.empty())
        {
            // Not expected while reading; do not spin on it.
            read_failed_ = true;
            connection_error_ = error;
            cv_.wait_for(lock, std::chrono::milliseconds(100), [this] { return shutdown_ || wake_.load(); });
        }
    }

    lock.unlock();
    if (backend_->isConnected())
        backend_->disconnect();
    clearState();
    lock.lock();
    status_ = Status::Disconnected;
}

void Controller::runTrajectory(std::uint64_t id, std::vector<Vec7> points, double dt)
{
    TrajectoryFollower follower(std::move(points), dt, limits_);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        index_ = 0;
        length_ = follower.length();
        time_ = 0.0;
        duration_ = follower.duration();
        stopping_ = false;
        settling_ = false;
        q_d_ = follower.firstPoint();
        dq_d_.fill(0.0);
    }

    runMotion(id, [&] {
        bool first = true;
        backend_->control([&](const Measured& measured, double period) {
            // The authoritative start check: execute() checked against a state
            // that may be a few milliseconds old.
            if (first)
            {
                first = false;
                for (int j = 0; j < kDof; ++j)
                {
                    const double gap = std::abs(follower.firstPoint()[j] - measured.q[j]);
                    if (gap > limits_.start_tolerance)
                        throw TrajectoryAbort(ErrorCodes::JointTrajectoryDoesNotStartAtCurrentPosition,
                                              "The robot moved before the trajectory started: it is " + num(gap) +
                                                  " rad from the start on q[" + std::to_string(j) +
                                                  "]. Plan again from the robot's current joint positions.");
                }
            }
            follower.step(period, measured.q, measured.dq, stop_requested_.load(std::memory_order_relaxed));
            checkTracking(follower.qDesired(), measured.q);
            Progress progress;
            progress.index = follower.index();
            progress.length = follower.length();
            progress.time = follower.time();
            progress.duration = follower.duration();
            progress.stopping = follower.interrupted();
            progress.settling = follower.phase() == TrajectoryFollower::Phase::Settling;
            progress.q_d = follower.qDesired();
            progress.dq_d = follower.dqDesired();
            publish(measured, &progress);
            Command command;
            command.q_d = follower.qDesired();
            command.dq_d = follower.dqDesired();
            command.finished = follower.phase() == TrajectoryFollower::Phase::Done;
            return command;
        });
        return follower.interrupted();
    });
}

void Controller::runJointMotion(std::uint64_t id, const Vec7& q_goal, double speed_factor)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        index_ = 0;
        length_ = 2;
        time_ = 0.0;
        duration_ = 0.0;
        stopping_ = false;
        settling_ = false;
        q_d_ = state_.q;
        dq_d_.fill(0.0);
    }

    runMotion(id, [&] {
        // Starts on the first cycle from where the robot holds its joints (q_d,
        // not the measured q), as libfranka's MotionGenerator does: the first
        // command then continues the robot's own, without a step.
        std::optional<JointMotionGenerator> generator;
        Progress progress;
        progress.length = 2;
        double rate = 1.0;
        bool arrived = false;
        backend_->controlJointPositions([&](const Measured& measured, double period) {
            if (!generator)
            {
                generator.emplace(measured.q_d, q_goal, speed_factor);
                progress.duration = generator->duration();
                progress.q_d = measured.q_d;
            }
            // Stopping slows the motion's clock down to rest over stop_time,
            // so the robot stops on the path, like a stopped trajectory.
            if (stop_requested_.load(std::memory_order_relaxed))
                progress.stopping = true;
            if (progress.stopping)
                rate = limits_.stop_time > 0.0 ? std::max(0.0, rate - period / limits_.stop_time) : 0.0;
            progress.time = std::min(progress.duration, progress.time + rate * period);
            Command command;
            arrived = generator->positions(progress.time, command.q_d);
            checkTracking(command.q_d, measured.q);
            for (int j = 0; j < kDof; ++j)
                progress.dq_d[j] = period > 0.0 ? (command.q_d[j] - progress.q_d[j]) / period : 0.0;
            progress.q_d = command.q_d;
            progress.index = arrived ? 1 : 0;
            publish(measured, &progress);
            command.finished = arrived || rate <= 0.0;
            return command;
        });
        return !arrived;
    });
}

void Controller::runMotion(std::uint64_t id, const std::function<bool()>& motion)
{
    ErrorCodes code = ErrorCodes::Success;
    std::string message;
    try
    {
        if (motion())
        {
            code = ErrorCodes::Interrupted;
            message = "The motion was stopped before its end";
        }
    }
    catch (const TrajectoryAbort& e)
    {
        code = e.code;
        message = e.what();
    }
    catch (const BackendNetworkError& e)
    {
        code = ErrorCodes::ConnectionLost;
        message = std::string("Connection to the robot lost: ") + e.what();
        connectionLost(e.what());
    }
    catch (const BackendControlError& e)
    {
        code = ErrorCodes::ControlException;
        message = std::string("The robot stopped the motion: ") + e.what();
    }
    catch (const std::exception& e)
    {
        code = ErrorCodes::ControlException;
        message = e.what();
    }
    finish(id, code, message);
}

// --- Safety: never chase a joint that cannot follow (old MAX_JOINT_DIFFERENCE_IMPEDENCE) --- //
void Controller::checkTracking(const Vec7& q_d, const Vec7& q) const
{
    for (int j = 0; j < kDof; ++j)
    {
        const double error = std::abs(q_d[j] - q[j]);
        if (error > limits_.tracking_error_limit)
            throw TrajectoryAbort(ErrorCodes::TrackingErrorTooLarge,
                                  "q[" + std::to_string(j) + "] was " + num(error) +
                                      " rad from its commanded position (limit " + num(limits_.tracking_error_limit) +
                                      " rad), so the motion was stopped. Check whether the robot hit "
                                      "something or was pushed.");
    }
}

void Controller::runRecovery(std::uint64_t id)
{
    ErrorCodes code = ErrorCodes::Success;
    std::string message;
    try
    {
        backend_->recover();
    }
    catch (const BackendNetworkError& e)
    {
        code = ErrorCodes::ConnectionLost;
        message = std::string("Connection to the robot lost: ") + e.what();
        connectionLost(e.what());
    }
    catch (const std::exception& e)
    {
        code = ErrorCodes::ControlException;
        message = std::string("Error recovery failed: ") + e.what();
    }
    finish(id, code, message);
}

void Controller::publish(const Measured& measured, const Progress* progress)
{
    std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
    if (!lock.owns_lock())
        return;
    state_ = measured;
    has_state_ = true;
    ++seq_;
    stamp_ = std::chrono::steady_clock::now();
    if (progress != nullptr)
    {
        index_ = progress->index;
        length_ = progress->length;
        time_ = progress->time;
        duration_ = progress->duration;
        stopping_ = progress->stopping;
        settling_ = progress->settling;
        q_d_ = progress->q_d;
        dq_d_ = progress->dq_d;
    }
}

void Controller::clearState()
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_state_ = false;
}

void Controller::connectionLost(const std::string& reason)
{
    backend_->disconnect();
    clearState();
    std::lock_guard<std::mutex> lock(mutex_);
    connection_error_ = "Connection to the robot lost: " + reason;
    status_ = Status::Disconnected;
    // A trajectory accepted before the loss must not start after a reconnect.
    finishPendingLocked(ErrorCodes::ConnectionLost, connection_error_);
}

void Controller::finish(std::uint64_t id, ErrorCodes code, const std::string& message)
{
    std::lock_guard<std::mutex> lock(mutex_);
    results_.push_back(Result{id, code, message});
    if (results_.size() > kMaxResults)
        results_.pop_front();
    // In the same breath as the result, so that no snapshot shows a finished
    // command still running.
    active_id_ = 0;
    if (status_ == Status::Executing || status_ == Status::Recovering)
        status_ = Status::Idle;
}

void Controller::finishPendingLocked(ErrorCodes code, const std::string& message)
{
    if (pending_ == Pending::None)
        return;
    results_.push_back(Result{pending_id_, code, message});
    if (results_.size() > kMaxResults)
        results_.pop_front();
    pending_ = Pending::None;
    pending_points_.clear();
}

} // namespace csc376_franky
