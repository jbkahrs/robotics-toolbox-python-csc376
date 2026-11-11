#include "csc376_franky/gripper_controller.hpp"

namespace csc376_franky
{

namespace
{

constexpr auto kReconnectInterval = std::chrono::seconds(2);
constexpr auto kPollInterval = std::chrono::milliseconds(50);

} // namespace

GripperController::GripperController(std::unique_ptr<GripperBackend> backend) : backend_(std::move(backend))
{
    thread_ = std::thread(&GripperController::run, this);
}

GripperController::~GripperController()
{
    shutdown();
}

void GripperController::shutdown()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        shutdown_ = true;
    }
    cv_.notify_all();
    if (thread_.joinable())
        thread_.join();
}

void GripperController::setConnectionWanted(bool wanted)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        want_connection_ = wanted;
    }
    cv_.notify_all();
}

template <typename F>
GripperController::Result GripperController::command(F&& f)
{
    std::unique_lock<std::mutex> guard(command_mutex_, std::try_to_lock);
    if (!guard.owns_lock())
        return Result{ErrorCodes::RobotBusy, false, "The gripper is already executing a command"};
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_)
        {
            std::string message = "The miniPC is not connected to the gripper";
            if (!connection_error_.empty())
                message += ": " + connection_error_;
            return Result{ErrorCodes::GripperNotConnected, false, message};
        }
    }
    busy_ = true;
    Result result;
    try
    {
        result.success = f();
    }
    catch (const BackendNetworkError& e)
    {
        result =
            Result{ErrorCodes::GripperNotConnected, false, std::string("Connection to the gripper lost: ") + e.what()};
        connectionLost(e.what());
    }
    catch (const std::exception& e)
    {
        result = Result{ErrorCodes::GripperCommandFailed, false, e.what()};
    }
    busy_ = false;
    return result;
}

GripperController::Result GripperController::homing()
{
    return command([this] { return backend_->homing(); });
}

GripperController::Result GripperController::move(double width, double speed)
{
    return command([&] { return backend_->move(width, speed); });
}

GripperController::Result GripperController::grasp(double width, double speed, double force, double epsilon_inner,
                                                   double epsilon_outer)
{
    return command([&] { return backend_->grasp(width, speed, force, epsilon_inner, epsilon_outer); });
}

GripperController::Result GripperController::stop()
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!connected_)
            return Result{ErrorCodes::GripperNotConnected, false, "The miniPC is not connected to the gripper"};
    }
    try
    {
        return Result{ErrorCodes::Success, backend_->stop(), ""};
    }
    catch (const BackendNetworkError& e)
    {
        connectionLost(e.what());
        return Result{ErrorCodes::GripperNotConnected, false,
                      std::string("Connection to the gripper lost: ") + e.what()};
    }
    catch (const std::exception& e)
    {
        return Result{ErrorCodes::GripperCommandFailed, false, e.what()};
    }
}

GripperController::Snapshot GripperController::snapshot() const
{
    Snapshot s;
    std::lock_guard<std::mutex> lock(mutex_);
    s.connected = connected_;
    s.connection_error = connection_error_;
    s.has_state = has_state_;
    s.state = state_;
    s.age = std::chrono::duration<double>(std::chrono::steady_clock::now() - stamp_).count();
    s.busy = busy_;
    return s;
}

void GripperController::connectionLost(const std::string& reason)
{
    backend_->disconnect();
    std::lock_guard<std::mutex> lock(mutex_);
    connected_ = false;
    has_state_ = false;
    connection_error_ = "Connection to the gripper lost: " + reason;
}

void GripperController::run()
{
    std::unique_lock<std::mutex> lock(mutex_);
    while (!shutdown_)
    {
        if (!want_connection_)
        {
            if (connected_)
            {
                lock.unlock();
                backend_->disconnect();
                lock.lock();
            }
            connected_ = false;
            has_state_ = false;
            connection_error_.clear();
            cv_.wait(lock, [this] { return shutdown_ || want_connection_; });
            continue;
        }

        if (!connected_)
        {
            lock.unlock();
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
                connection_error_ = error;
                cv_.wait_for(lock, kReconnectInterval, [this] { return shutdown_ || !want_connection_; });
                continue;
            }
            connected_ = true;
            connection_error_.clear();
        }

        lock.unlock();
        try
        {
            const GripperSample sample = backend_->readOnce();
            lock.lock();
            state_ = sample;
            has_state_ = true;
            stamp_ = std::chrono::steady_clock::now();
        }
        catch (const BackendNetworkError& e)
        {
            connectionLost(e.what());
            lock.lock();
        }
        catch (const std::exception& e)
        {
            lock.lock();
            connection_error_ = e.what();
        }
        cv_.wait_for(lock, kPollInterval, [this] { return shutdown_ || !want_connection_; });
    }

    lock.unlock();
    if (backend_->isConnected())
        backend_->disconnect();
}

} // namespace csc376_franky
