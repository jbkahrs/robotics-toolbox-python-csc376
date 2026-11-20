#include "csc376_franky/motion_log.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <ctime>
#include <system_error>
#include <utility>

namespace csc376_franky
{

namespace
{

// The ring buffer holds this many events: 8 s of samples at 1 kHz. A power of two.
constexpr std::size_t kRingSize = 8192;
constexpr std::size_t kRingMask = kRingSize - 1;
// How often the log's thread writes what has come in.
constexpr auto kWritePeriod = std::chrono::milliseconds(20);
// How long begin() and end() wait for room in the ring before giving up: the
// ring only fills when the log's thread has been stuck for its 8 s, so a
// long wait would hold the controller's thread up for nothing.
constexpr auto kControlTimeout = std::chrono::milliseconds(200);
constexpr const char* kPrefix = "motion-";
constexpr const char* kSuffix = ".csv";

constexpr const char* kColumnsComment =
    "# columns: time [s] from the motion's first cycle, on the robot's clock; period [s] since the previous cycle "
    "(0.001, more where the robot lost cycles); mode (franka::RobotMode); success_rate "
    "(control_command_success_rate); index, the trajectory point passed; phase (0 running, 1 stopping, "
    "2 settling); q, dq (measured), q_d, dq_d (commanded), tau_J (measured), tau_J_d (the torque the robot was "
    "asked for), tau_ext (tau_ext_hat_filtered) for joints 0 to 6 [rad, rad/s, Nm]; x, y, z of the end effector "
    "in the base frame [m]; vx .. wz, the twist followed under velocity control, in the base frame [m/s, rad/s], "
    "and limited, the TwistFollower::Limited bits holding it back\n";

constexpr const char* kColumns = "time,period,mode,success_rate,index,phase,"
                                 "q0,q1,q2,q3,q4,q5,q6,dq0,dq1,dq2,dq3,dq4,dq5,dq6,"
                                 "q_d0,q_d1,q_d2,q_d3,q_d4,q_d5,q_d6,dq_d0,dq_d1,dq_d2,dq_d3,dq_d4,dq_d5,dq_d6,"
                                 "tau_J0,tau_J1,tau_J2,tau_J3,tau_J4,tau_J5,tau_J6,"
                                 "tau_J_d0,tau_J_d1,tau_J_d2,tau_J_d3,tau_J_d4,tau_J_d5,tau_J_d6,"
                                 "tau_ext0,tau_ext1,tau_ext2,tau_ext3,tau_ext4,tau_ext5,tau_ext6,"
                                 "x,y,z,vx,vy,vz,wx,wy,wz,limited\n";

// Local time like 2026-11-19T14:30:22.123-0500, or 20261119-143022 for a file name.
std::string timestamp(std::chrono::system_clock::time_point now, bool for_name)
{
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
    std::tm local{};
    localtime_r(&seconds, &local);
    char buffer[64];
    if (for_name)
    {
        std::strftime(buffer, sizeof buffer, "%Y%m%d-%H%M%S", &local);
        return buffer;
    }
    std::strftime(buffer, sizeof buffer, "%Y-%m-%dT%H:%M:%S", &local);
    std::string text = buffer;
    std::snprintf(buffer, sizeof buffer, ".%03d", static_cast<int>(millis));
    text += buffer;
    std::strftime(buffer, sizeof buffer, "%z", &local);
    return text + buffer;
}

void append(std::string& line, const char* format, double value)
{
    char buffer[32];
    std::snprintf(buffer, sizeof buffer, format, value);
    line += buffer;
}

void appendAll(std::string& line, const double* values, std::size_t count)
{
    for (std::size_t i = 0; i < count; ++i)
    {
        line += ',';
        append(line, "%.7g", values[i]);
    }
}

} // namespace

MotionLog::MotionLog(std::filesystem::path directory, std::uintmax_t max_bytes, std::string preamble)
    : directory_(std::move(directory)), max_bytes_(max_bytes), preamble_(std::move(preamble)), ring_(kRingSize)
{
    line_.reserve(1024);
    std::error_code ec;
    std::filesystem::create_directories(directory_, ec);
    if (ec)
        error_ = "cannot create " + directory_.string() + ": " + ec.message();
    thread_ = std::thread(&MotionLog::run, this);
}

MotionLog::~MotionLog()
{
    shutdown();
}

void MotionLog::shutdown()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable())
        thread_.join();
}

std::string MotionLog::error() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return error_;
}

std::uint64_t MotionLog::files() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return files_;
}

std::uint64_t MotionLog::dropped() const
{
    return dropped_.load(std::memory_order_relaxed);
}

// --- The controller's thread --- //

bool MotionLog::push(Kind kind, const LogSample* sample)
{
    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) >= kRingSize)
        return false;
    Event& event = ring_[head & kRingMask];
    event.kind = kind;
    if (sample != nullptr)
        event.sample = *sample;
    head_.store(head + 1, std::memory_order_release);
    return true;
}

bool MotionLog::post(Kind kind)
{
    const auto deadline = std::chrono::steady_clock::now() + kControlTimeout;
    while (!push(kind, nullptr))
    {
        if (std::chrono::steady_clock::now() > deadline)
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return true;
}

std::filesystem::path MotionLog::begin(std::uint64_t id, const std::string& kind, const std::string& fields)
{
    const auto now = std::chrono::system_clock::now();
    char number[32];
    std::snprintf(number, sizeof number, "%06llu", static_cast<unsigned long long>(id));
    const std::filesystem::path path =
        directory_ / (kPrefix + timestamp(now, true) + "-" + number + "-" + kind + kSuffix);
    std::string text = "# motion: {\"id\": " + std::to_string(id) + ", \"kind\": " + jsonString(kind) +
                       ", \"started\": " + jsonString(timestamp(now, false));
    if (!fields.empty())
        text += ", " + fields;
    text += "}\n";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.push_back(Pending{path, std::move(text)});
    }
    motion_samples_ = 0;
    motion_dropped_ = 0;
    if (!post(Kind::Begin))
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.pop_back();
        error_ = "the motion log is not keeping up: motion " + std::to_string(id) + " is not logged";
        return {};
    }
    return path;
}

void MotionLog::record(const LogSample& sample)
{
    if (push(Kind::Sample, &sample))
    {
        ++motion_samples_;
        return;
    }
    ++motion_dropped_;
    dropped_.fetch_add(1, std::memory_order_relaxed);
}

void MotionLog::end(const std::string& fields)
{
    std::string text = "# result: {";
    if (!fields.empty())
        text += fields + ", ";
    text +=
        "\"samples\": " + std::to_string(motion_samples_) + ", \"dropped\": " + std::to_string(motion_dropped_) + "}\n";
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.push_back(Pending{{}, std::move(text)});
    }
    if (!post(Kind::End))
    {
        std::lock_guard<std::mutex> lock(mutex_);
        pending_.pop_back();
        error_ = "the motion log is not keeping up: a motion's result was not logged";
    }
}

// --- The log's thread --- //

void MotionLog::run()
{
    std::unique_lock<std::mutex> lock(mutex_);
    while (true)
    {
        cv_.wait_for(lock, kWritePeriod, [this] { return shutdown_; });
        const bool last = shutdown_;
        lock.unlock();
        drain();
        if (last)
            break;
        lock.lock();
    }
    if (file_ != nullptr)
        close("# result: {\"message\": \"the motion log was shut down before the motion ended\"}\n");
}

void MotionLog::drain()
{
    std::size_t tail = tail_.load(std::memory_order_relaxed);
    const std::size_t head = head_.load(std::memory_order_acquire);
    while (tail != head)
    {
        const Event& event = ring_[tail & kRingMask];
        if (event.kind == Kind::Sample)
        {
            write(event.sample);
        }
        else
        {
            Pending pending;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!pending_.empty())
                {
                    pending = std::move(pending_.front());
                    pending_.pop_front();
                }
            }
            if (event.kind == Kind::Begin)
                open(pending);
            else
                close(pending.text);
        }
        ++tail;
        tail_.store(tail, std::memory_order_release);
    }
}

void MotionLog::open(const Pending& pending)
{
    if (file_ != nullptr)
        close("# result: {\"message\": \"the next motion started before this one's result was logged\"}\n");
    first_ = true;
    failed_ = false;
    if (pending.path.empty())
        return;
    std::error_code ec;
    std::filesystem::create_directories(directory_, ec);
    file_ = std::fopen(pending.path.c_str(), "w");
    if (file_ == nullptr)
    {
        fail("cannot create " + pending.path.string() + ": " + std::strerror(errno));
        return;
    }
    std::setvbuf(file_, nullptr, _IOFBF, 1 << 16);
    path_ = pending.path;
    std::fputs(preamble_.c_str(), file_);
    if (!preamble_.empty() && preamble_.back() != '\n')
        std::fputc('\n', file_);
    std::fputs(pending.text.c_str(), file_);
    std::fputs(kColumnsComment, file_);
    std::fputs(kColumns, file_);
}

void MotionLog::write(const LogSample& sample)
{
    if (file_ == nullptr || failed_)
        return;
    if (first_)
    {
        first_ = false;
        start_time_ = sample.time;
    }
    line_.clear();
    append(line_, "%.3f", sample.time - start_time_);
    line_ += ',';
    append(line_, "%.3f", sample.period);
    line_ += ',';
    line_ += std::to_string(sample.robot_mode);
    line_ += ',';
    append(line_, "%.3f", sample.success_rate);
    line_ += ',';
    line_ += std::to_string(sample.index);
    line_ += ',';
    line_ += std::to_string(sample.phase);
    appendAll(line_, sample.q.data(), kDof);
    appendAll(line_, sample.dq.data(), kDof);
    appendAll(line_, sample.q_d.data(), kDof);
    appendAll(line_, sample.dq_d.data(), kDof);
    appendAll(line_, sample.tau_J.data(), kDof);
    appendAll(line_, sample.tau_J_d.data(), kDof);
    appendAll(line_, sample.tau_ext.data(), kDof);
    appendAll(line_, sample.position.data(), sample.position.size());
    appendAll(line_, sample.twist.data(), sample.twist.size());
    line_ += ',';
    line_ += std::to_string(sample.limited);
    line_ += '\n';
    if (std::fwrite(line_.data(), 1, line_.size(), file_) != line_.size())
    {
        failed_ = true;
        fail("cannot write " + path_.string() + ": " + std::strerror(errno));
    }
}

void MotionLog::close(const std::string& trailer)
{
    if (file_ == nullptr)
        return;
    std::fputs(trailer.c_str(), file_);
    const bool failed = std::ferror(file_) != 0;
    const bool closed = std::fclose(file_) == 0;
    const int close_error = errno;
    file_ = nullptr;
    if (!failed_ && !closed)
        fail("cannot write " + path_.string() + ": " + std::strerror(close_error));
    else if (!failed_ && failed)
        fail("cannot write " + path_.string());
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++files_;
    }
    prune();
}

// Removes the oldest motion logs while they take more than max_bytes
// together. Never the file just written, whatever its size.
void MotionLog::prune()
{
    struct File
    {
        std::filesystem::file_time_type time;
        std::string name;
        std::uintmax_t size;
    };
    const std::string prefix = kPrefix;
    const std::string suffix = kSuffix;
    const std::string current = path_.filename().string();
    std::vector<File> files;
    std::uintmax_t total = 0;
    std::error_code ec;
    for (std::filesystem::directory_iterator it(directory_, ec), end; !ec && it != end; it.increment(ec))
    {
        const std::string name = it->path().filename().string();
        if (name.size() <= prefix.size() + suffix.size() || name.compare(0, prefix.size(), prefix) != 0 ||
            name.compare(name.size() - suffix.size(), suffix.size(), suffix) != 0)
            continue;
        std::error_code ignored;
        if (!it->is_regular_file(ignored))
            continue;
        const std::uintmax_t size = it->file_size(ignored);
        if (ignored)
            continue;
        const std::filesystem::file_time_type time = it->last_write_time(ignored);
        if (ignored)
            continue;
        total += size;
        if (name != current)
            files.push_back(File{time, name, size});
    }
    // Oldest first, by when they were written: the names carry local time,
    // which a clock change can put out of order.
    std::sort(files.begin(), files.end(),
              [](const File& a, const File& b) { return a.time != b.time ? a.time < b.time : a.name < b.name; });
    for (std::size_t i = 0; i < files.size() && total > max_bytes_; ++i)
    {
        std::error_code ignored;
        if (std::filesystem::remove(directory_ / files[i].name, ignored))
            total -= files[i].size;
    }
}

void MotionLog::fail(const std::string& what)
{
    std::lock_guard<std::mutex> lock(mutex_);
    error_ = what;
}

} // namespace csc376_franky
