#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "csc376_franky/backend.hpp"

namespace csc376_franky
{

// One control cycle of a motion, as the motion log records it.
struct LogSample
{
    // The robot's clock [s]; the log writes it from the motion's first cycle.
    double time = 0.0;
    // Since the previous cycle [s]: 0.001, or more where the robot lost cycles.
    double period = 0.0;
    int robot_mode = 0;
    double success_rate = 0.0;
    // The trajectory point passed.
    std::size_t index = 0;
    // 0 running, 1 stopping, 2 settling.
    int phase = 0;
    Vec7 q{};
    Vec7 dq{};
    Vec7 q_d{};
    Vec7 dq_d{};
    Vec7 tau_J{};
    Vec7 tau_J_d{};
    Vec7 tau_ext{};
    // The end effector's position in the base frame [m].
    std::array<double, 3> position{};
    // Under velocity control: the twist followed, in the base frame, and the
    // TwistFollower::Limited bits that hold it back.
    Vec6 twist{};
    unsigned limited = 0;
};

// Records every control cycle of every motion, for tuning the controller: one
// CSV file per motion in directory, named motion-<date>-<time>-<id>-<kind>.csv,
// the oldest removed once they take more than max_bytes together.
//
// A file starts with "# " lines (the preamble given here, then the motion and
// the columns), then the column names and one line per cycle, and ends with
// "# result: {...}". record() is called from the 1 kHz control callback: it
// copies the sample into a ring buffer without locking or allocating, and a
// thread of its own writes the files. A sample is dropped, and counted, when
// that thread falls behind.
class MotionLog
{
public:
    MotionLog(std::filesystem::path directory, std::uintmax_t max_bytes, std::string preamble);
    ~MotionLog();
    MotionLog(const MotionLog&) = delete;
    MotionLog& operator=(const MotionLog&) = delete;

    // Starts the file of a motion; fields are JSON members for its "motion"
    // line, e.g. "points": 301. Returns the file's path, empty if it could
    // not be started. From the controller's thread, outside the control loop.
    std::filesystem::path begin(std::uint64_t id, const std::string& kind, const std::string& fields);
    // From the control callback: no locks, no allocation.
    void record(const LogSample& sample);
    // Ends the motion's file; fields are the JSON members of its "result" line.
    void end(const std::string& fields);

    const std::filesystem::path& directory() const { return directory_; }
    // What last went wrong writing, or empty.
    std::string error() const;
    // Files finished, and samples dropped, since the start.
    std::uint64_t files() const;
    std::uint64_t dropped() const;
    // Writes what is queued and ends the thread.
    void shutdown();

private:
    enum class Kind
    {
        Begin,
        Sample,
        End,
    };

    struct Event
    {
        Kind kind = Kind::Sample;
        LogSample sample;
    };

    // What goes with a Begin (the file and its "motion" line) or an End (the
    // "result" line), in the order of their events.
    struct Pending
    {
        std::filesystem::path path;
        std::string text;
    };

    bool push(Kind kind, const LogSample* sample);
    // Pushes a Begin or End, waiting for room; false if there was none in
    // time. The log's thread picks it up with its next write, within kWritePeriod.
    bool post(Kind kind);
    void run();
    void drain();
    void open(const Pending& pending);
    void close(const std::string& trailer);
    void write(const LogSample& sample);
    void prune();
    void fail(const std::string& what);

    const std::filesystem::path directory_;
    const std::uintmax_t max_bytes_;
    const std::string preamble_;

    // The ring buffer: the controller's thread writes at head_, the log's thread reads at tail_.
    std::vector<Event> ring_;
    std::atomic<std::size_t> head_{0};
    std::atomic<std::size_t> tail_{0};
    // The current motion's samples and dropped samples; the controller's thread only.
    std::uint64_t motion_samples_ = 0;
    std::uint64_t motion_dropped_ = 0;
    std::atomic<std::uint64_t> dropped_{0};

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool shutdown_ = false;
    std::deque<Pending> pending_;
    std::string error_;
    std::uint64_t files_ = 0;

    // The log's thread only.
    std::FILE* file_ = nullptr;
    std::filesystem::path path_;
    std::string line_;
    bool first_ = true;
    double start_time_ = 0.0;
    bool failed_ = false;

    std::thread thread_;
};

} // namespace csc376_franky
