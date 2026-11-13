#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "csc376_franky/backend.hpp"

namespace csc376_franky
{

// Keeps the Franka Hand connected while wanted and polls its state from a
// thread of its own. Commands block the calling thread and run one at a
// time; stop() can always interrupt the one that is running.
class GripperController
{
public:
    struct Result
    {
        ErrorCodes code = ErrorCodes::Success;
        // What the hand reported: false, for example, if a grasp found no object.
        bool success = false;
        std::string message;
    };

    struct Snapshot
    {
        bool connected = false;
        std::string connection_error;
        bool has_state = false;
        double age = 0.0;
        GripperSample state;
        bool busy = false;
    };

    explicit GripperController(std::unique_ptr<GripperBackend> backend);
    ~GripperController();
    GripperController(const GripperController&) = delete;
    GripperController& operator=(const GripperController&) = delete;

    void setConnectionWanted(bool wanted);
    Result homing();
    Result move(double width, double speed);
    Result grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer);
    Result stop();
    Snapshot snapshot() const;
    GripperBackend& backend() { return *backend_; }
    void shutdown();

private:
    template <typename F>
    Result command(F&& f);
    void run();
    void connectionLost(const std::string& reason);

    std::unique_ptr<GripperBackend> backend_;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool shutdown_ = false;
    bool want_connection_ = false;
    bool connected_ = false;
    std::string connection_error_;
    bool has_state_ = false;
    GripperSample state_;
    std::chrono::steady_clock::time_point stamp_{};

    std::mutex command_mutex_;
    std::atomic<bool> busy_{false};
    std::thread thread_;
};

} // namespace csc376_franky
