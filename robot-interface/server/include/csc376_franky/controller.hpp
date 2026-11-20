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
#include "csc376_franky/motion_log.hpp"
#include "csc376_franky/trajectory.hpp"
#include "csc376_franky/twist_follower.hpp"

namespace csc376_franky
{

// Drives one robot from a thread of its own. While a connection is wanted it
// keeps the robot connected and streams its state; it runs one motion (a
// trajectory, a joint motion or velocity control), or one error recovery, at
// a time, and refuses anything else meanwhile.
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

    // What the robot is executing.
    enum class Motion
    {
        None,
        Trajectory,
        JointMotion,
        Servo,
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
        // The motion log's file for it, if there is a log.
        std::string log_file;
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
        // The running motion, if active_id is not 0.
        std::uint64_t active_id = 0;
        Motion motion = Motion::None;
        std::size_t index = 0;
        std::size_t length = 0;
        double time = 0.0;
        double duration = 0.0;
        bool stopping = false;
        // Past the last point, holding it until the robot is at rest.
        bool settling = false;
        Vec7 q_d{};
        Vec7 dq_d{};
        // Under velocity control: the twist being followed, in the base frame,
        // and the TwistFollower::Limited bits that hold it back.
        Vec6 twist{};
        unsigned limited = 0;
        // The most recently finished commands, oldest first.
        std::vector<Result> results;
    };

    // With a log, every motion is recorded there at the control rate.
    Controller(std::unique_ptr<Backend> backend, const Limits& limits, std::shared_ptr<MotionLog> log = nullptr);
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
    // Accepts velocity control, like execute(): the end effector then follows
    // the twist of setServoTwist(), within the limits (see TwistFollower),
    // until endServo(), stop() or an error ends it.
    Submission startServo();
    // The twist for velocity control to follow from now on. id is the
    // Submission's: it fails unless that velocity control is running or about
    // to start.
    ValidationResult setServoTwist(std::uint64_t id, const Vec6& twist, TwistFrame frame);
    // Slows that velocity control down to rest and ends it; unlike after
    // stop(), its result is a success.
    void endServo(std::uint64_t id);
    // Clears a reflex or other error (franka::Robot::automaticErrorRecovery).
    Submission recover();
    // Slows the running motion to rest (a trajectory along its path), and
    // drops a motion that has not started yet.
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
        Servo,
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
        bool settling = false;
        Vec7 q_d{};
        Vec7 dq_d{};
        Vec6 twist{};
        unsigned limited = 0;
    };

    // The twist that velocity control is to follow, as setServoTwist() left it.
    struct ServoTarget
    {
        Vec6 twist{};
        TwistFrame frame = TwistFrame::EndEffector;
        // Counts the calls of setServoTwist(), so that a new target is told from the old one.
        std::uint64_t seq = 0;
    };

    void run();
    void runTrajectory(std::uint64_t id, std::vector<Vec7> points, double dt);
    void runJointMotion(std::uint64_t id, const Vec7& q_goal, double speed_factor);
    void runServo(std::uint64_t id);
    // Runs a motion (which returns true if it was stopped early) and records
    // how it ended. kind and fields (JSON members) describe it to the log.
    void runMotion(std::uint64_t id, const char* kind, const std::string& fields, const std::function<bool()>& motion);
    // Gives the log one control cycle.
    void record(const Measured& measured, double period, const Progress& progress);
    void checkTracking(const Vec7& q_d, const Vec7& q) const;
    void runRecovery(std::uint64_t id);
    void publish(const Measured& measured, const Progress* progress);
    void clearState();
    void connectionLost(const std::string& reason);
    void finish(std::uint64_t id, ErrorCodes code, const std::string& message, const std::string& log_file);
    void finishPendingLocked(ErrorCodes code, const std::string& message);
    bool busyLocked() const;
    // Whether the velocity control with this id is the running motion.
    bool servoRunningLocked(std::uint64_t id) const;
    // Whether a motion can start now; if so, state is the robot's latest state.
    ErrorCodes readyLocked(std::string& message, Measured& state) const;
    ErrorCodes notConnectedLocked(std::string& message) const;

    std::unique_ptr<Backend> backend_;
    const Limits limits_;
    const std::shared_ptr<MotionLog> log_;

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
    Motion active_motion_ = Motion::None;
    std::deque<Result> results_;
    // The last read ended in an error that connection_error_ still shows.
    // Used by the controller's thread only.
    bool read_failed_ = false;

    // Ends the read loop so the thread can pick up a command.
    std::atomic<bool> wake_{false};
    std::atomic<bool> stop_requested_{false};
    // endServo() was called: velocity control is to end, successfully.
    std::atomic<bool> servo_end_requested_{false};

    // Read from the 1 kHz callback with try_lock, so it never waits on a writer.
    std::mutex servo_mutex_;
    ServoTarget servo_target_;

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
    bool settling_ = false;
    Vec7 q_d_{};
    Vec7 dq_d_{};
    Vec6 twist_{};
    unsigned limited_ = 0;

    std::thread thread_;
};

} // namespace csc376_franky
