#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "csc376_franky/backend.hpp"
#include "csc376_franky/trajectory.hpp"

namespace csc376_franky
{

// Drives one robot from a thread of its own. While a connection is wanted it
// keeps the robot connected and streams its state; it runs one trajectory, or
// one error recovery, at a time, and refuses anything else meanwhile.
class Controller
{
public:
    enum class Status
    {
        Disconnected,
        Connecting,
        Idle,
        Executing,
        Recovering,
    };

    struct Submission
    {
        ErrorCodes code = ErrorCodes::Success;
        std::string message;
        std::uint64_t id = 0;
    };

    // How an accepted command (identified by its Submission id) ended.
    struct Result
    {
        std::uint64_t id = 0;
        ErrorCodes code = ErrorCodes::Success;
        std::string message;
    };

    struct Snapshot
    {
        Status status = Status::Disconnected;
        bool connected = false;
        std::string connection_error;
        bool has_state = false;
        std::uint64_t seq = 0;
        double age = 0.0;
        Measured state;
        std::vector<std::string> current_errors;
        std::vector<std::string> last_motion_errors;
        // The running trajectory, if active_id is not 0.
        std::uint64_t active_id = 0;
        std::size_t index = 0;
        std::size_t length = 0;
        double time = 0.0;
        double duration = 0.0;
        bool stopping = false;
        Vec7 q_d{};
        Vec7 dq_d{};
        // The most recently finished commands, oldest first.
        std::vector<Result> results;
    };

    Controller(std::unique_ptr<Backend> backend, const Limits& limits);
    ~Controller();
    Controller(const Controller&) = delete;
    Controller& operator=(const Controller&) = delete;

    // Connect to the robot while true; disconnect (stopping any motion) when false.
    void setConnectionWanted(bool wanted);
    // Checks a trajectory against the limits and the measured joint positions.
    ValidationResult validate(const std::vector<Vec7>& points, double dt) const;
    // Accepts a trajectory if the robot is idle and ready and the trajectory
    // validates; it then starts right away. Its result shows up in snapshot().
    Submission execute(std::vector<Vec7> points, double dt);
    // Accepts a move to q_goal, like execute(), as libfranka's examples move:
    // their MotionGenerator, through the robot's joint position interface.
    // speed_factor may be up to limits().velocity_scale.
    Submission moveToJointPositions(const Vec7& q_goal, double speed_factor);
    // Clears a reflex or other error (franka::Robot::automaticErrorRecovery).
    Submission recover();
    // Slows the running trajectory to rest along its path, and drops a
    // trajectory that has not started yet.
    void stop();
    Snapshot snapshot() const;
    const Limits& limits() const { return limits_; }
    Backend& backend() { return *backend_; }
    // Stops any motion and ends the thread.
    void shutdown();

private:
    enum class Pending
    {
        None,
        Execute,
        JointMotion,
        Recover,
    };

    // What publish() shows of the running motion.
    struct Progress
    {
        std::size_t index = 0;
        std::size_t length = 0;
        double time = 0.0;
        double duration = 0.0;
        bool stopping = false;
        Vec7 q_d{};
        Vec7 dq_d{};
    };

    void run();
    void runTrajectory(std::uint64_t id, std::vector<Vec7> points, double dt);
    void runJointMotion(std::uint64_t id, const Vec7& q_goal, double speed_factor);
    // Runs a motion (which returns true if it was stopped early) and records how it ended.
    void runMotion(std::uint64_t id, const std::function<bool()>& motion);
    void checkTracking(const Vec7& q_d, const Vec7& q) const;
    void runRecovery(std::uint64_t id);
    void publish(const Measured& measured, const Progress* progress);
    void clearState();
    void connectionLost(const std::string& reason);
    void finish(std::uint64_t id, ErrorCodes code, const std::string& message);
    void finishPendingLocked(ErrorCodes code, const std::string& message);
    bool busyLocked() const;
    // Whether a motion can start now; if so, state is the robot's latest state.
    ErrorCodes readyLocked(std::string& message, Measured& state) const;
    ErrorCodes notConnectedLocked(std::string& message) const;

    std::unique_ptr<Backend> backend_;
    const Limits limits_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool shutdown_ = false;
    bool want_connection_ = false;
    Status status_ = Status::Disconnected;
    std::string connection_error_;
    Pending pending_ = Pending::None;
    std::uint64_t pending_id_ = 0;
    std::vector<Vec7> pending_points_;
    double pending_dt_ = 0.0;
    Vec7 pending_goal_{};
    double pending_speed_factor_ = 0.0;
    std::uint64_t next_id_ = 1;
    std::uint64_t active_id_ = 0;
    std::deque<Result> results_;

    // Ends the read loop so the thread can pick up a command.
    std::atomic<bool> wake_{false};
    std::atomic<bool> stop_requested_{false};

    // Written from the 1 kHz callbacks with try_lock, so they never wait on a reader.
    mutable std::mutex state_mutex_;
    bool has_state_ = false;
    std::uint64_t seq_ = 0;
    std::chrono::steady_clock::time_point stamp_{};
    Measured state_;
    std::size_t index_ = 0;
    std::size_t length_ = 0;
    double time_ = 0.0;
    double duration_ = 0.0;
    bool stopping_ = false;
    Vec7 q_d_{};
    Vec7 dq_d_{};

    std::thread thread_;
};

} // namespace csc376_franky
