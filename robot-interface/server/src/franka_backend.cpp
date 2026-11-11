#include <algorithm>
#include <mutex>

#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/lowpass_filter.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "csc376_franky/backend.hpp"

namespace csc376_franky
{

namespace
{

// franka::Errors converts to ["name", "name"]; split that back into names.
std::vector<std::string> errorNames(const franka::Errors& errors)
{
    std::vector<std::string> names;
    std::string name;
    for (const char c : static_cast<std::string>(errors))
    {
        if (c == ',' || c == ']')
        {
            if (!name.empty())
                names.push_back(name);
            name.clear();
        }
        else if (c != '[' && c != '"' && c != ' ')
        {
            name += c;
        }
    }
    return names;
}

Measured toMeasured(const franka::RobotState& state)
{
    Measured m;
    m.q = state.q;
    m.dq = state.dq;
    m.q_d = state.q_d;
    m.tau_J = state.tau_J;
    m.tau_ext_hat_filtered = state.tau_ext_hat_filtered;
    m.O_T_EE = state.O_T_EE;
    m.O_F_ext_hat_K = state.O_F_ext_hat_K;
    m.elbow = state.elbow;
    m.robot_mode = static_cast<int>(state.robot_mode);
    m.time = state.time.toSec();
    m.control_command_success_rate = state.control_command_success_rate;
    return m;
}

class FrankaBackend : public Backend
{
public:
    FrankaBackend(std::string robot_ip, const Limits& limits, bool enforce_realtime)
        : robot_ip_(std::move(robot_ip)), stiffness_(limits.stiffness), damping_(limits.damping),
          enforce_realtime_(enforce_realtime)
    {
    }

    std::string name() const override { return "franka"; }
    std::string address() const override { return robot_ip_; }

    void connect() override
    {
        try
        {
            auto robot = std::make_unique<franka::Robot>(
                robot_ip_, enforce_realtime_ ? franka::RealtimeConfig::kEnforce : franka::RealtimeConfig::kIgnore);
            model_ = std::make_unique<franka::Model>(robot->loadModel());
            robot_ = std::move(robot);
        }
        catch (const franka::Exception& e)
        {
            model_.reset();
            throw BackendNetworkError(e.what());
        }
        configured_ = false;
        try
        {
            configure();
        }
        catch (const BackendControlError&)
        {
            // The robot refuses to be configured while it cannot move (user stop
            // pressed, reflex). It is connected all the same, so its state says
            // why; configure() runs again before the first motion.
        }
    }

    void disconnect() override
    {
        model_.reset();
        robot_.reset();
        configured_ = false;
    }

    bool isConnected() const override { return robot_ != nullptr; }

    void read(const ReadCallback& callback) override
    {
        requireConnection();
        try
        {
            robot_->read([&](const franka::RobotState& state) {
                storeErrors(state);
                return callback(toMeasured(state));
            });
        }
        catch (const franka::NetworkException& e)
        {
            disconnect();
            throw BackendNetworkError(e.what());
        }
        catch (const franka::Exception& e)
        {
            throw BackendControlError(e.what());
        }
    }

    void control(const ControlCallback& callback) override
    {
        requireConnection();
        configure();
        try
        {
            // limit_rate and the cutoff frequency are libfranka 0.9's defaults,
            // spelled out because later versions default to neither.
            robot_->control(
                [&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
                    storeErrors(state);
                    const Command command = callback(toMeasured(state), period.toSec());
                    const std::array<double, 7> coriolis = model_->coriolis(state);
                    std::array<double, 7> tau{};
                    for (int i = 0; i < kDof; ++i)
                    {
                        tau[i] = stiffness_[i] * (command.q_d[i] - state.q[i]) +
                                 damping_[i] * (command.dq_d[i] - state.dq[i]) + coriolis[i];
                    }
                    const franka::Torques torques(franka::limitRate(franka::kMaxTorqueRate, tau, state.tau_J_d));
                    return command.finished ? franka::MotionFinished(torques) : torques;
                },
                true, franka::kDefaultCutoffFrequency);
        }
        catch (const TrajectoryAbort&)
        {
            throw;
        }
        catch (const franka::NetworkException& e)
        {
            disconnect();
            throw BackendNetworkError(e.what());
        }
        catch (const franka::Exception& e)
        {
            throw BackendControlError(e.what());
        }
    }

    void controlJointPositions(const ControlCallback& callback) override
    {
        requireConnection();
        configure();
        try
        {
            robot_->control(
                [&](const franka::RobotState& state, franka::Duration period) -> franka::JointPositions {
                    storeErrors(state);
                    const Command command = callback(toMeasured(state), period.toSec());
                    const franka::JointPositions positions(command.q_d);
                    return command.finished ? franka::MotionFinished(positions) : positions;
                },
                franka::ControllerMode::kJointImpedance, true, franka::kDefaultCutoffFrequency);
        }
        catch (const TrajectoryAbort&)
        {
            throw;
        }
        catch (const franka::NetworkException& e)
        {
            disconnect();
            throw BackendNetworkError(e.what());
        }
        catch (const franka::Exception& e)
        {
            throw BackendControlError(e.what());
        }
    }

    void recover() override
    {
        requireConnection();
        try
        {
            robot_->automaticErrorRecovery();
        }
        catch (const franka::NetworkException& e)
        {
            disconnect();
            throw BackendNetworkError(e.what());
        }
        catch (const franka::Exception& e)
        {
            throw BackendControlError(e.what());
        }
    }

    void errors(std::vector<std::string>& current, std::vector<std::string>& last_motion) const override
    {
        std::lock_guard<std::mutex> lock(errors_mutex_);
        current = errorNames(current_errors_);
        last_motion = errorNames(last_motion_errors_);
    }

private:
    void requireConnection() const
    {
        if (!robot_)
            throw BackendNetworkError("not connected to the robot");
    }

    // Sets the collision thresholds and the joint impedance, once per connection.
    void configure()
    {
        if (configured_)
            return;
        try
        {
            // The old controller's collision thresholds [Nm], [N].
            // clang-format off
            robot_->setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
            // clang-format on
            // The robot's own joint impedance, for controlJointPositions (libfranka's
            // setDefaultBehavior); the torque loop in control() does not use it.
            robot_->setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
            configured_ = true;
        }
        catch (const franka::NetworkException& e)
        {
            disconnect();
            throw BackendNetworkError(e.what());
        }
        catch (const franka::Exception& e)
        {
            throw BackendControlError(e.what());
        }
    }

    // Called at 1 kHz: never waits for a reader, and copies without allocating.
    void storeErrors(const franka::RobotState& state)
    {
        std::unique_lock<std::mutex> lock(errors_mutex_, std::try_to_lock);
        if (lock.owns_lock())
        {
            current_errors_ = state.current_errors;
            last_motion_errors_ = state.last_motion_errors;
        }
    }

    const std::string robot_ip_;
    const Vec7 stiffness_;
    const Vec7 damping_;
    const bool enforce_realtime_;
    std::unique_ptr<franka::Robot> robot_;
    std::unique_ptr<franka::Model> model_;
    bool configured_ = false;
    mutable std::mutex errors_mutex_;
    franka::Errors current_errors_;
    franka::Errors last_motion_errors_;
};

class FrankaGripperBackend : public GripperBackend
{
public:
    explicit FrankaGripperBackend(std::string robot_ip) : robot_ip_(std::move(robot_ip)) {}

    std::string name() const override { return "franka"; }

    void connect() override
    {
        try
        {
            auto gripper = std::make_shared<franka::Gripper>(robot_ip_);
            std::lock_guard<std::mutex> lock(mutex_);
            gripper_ = std::move(gripper);
        }
        catch (const franka::Exception& e)
        {
            throw BackendNetworkError(e.what());
        }
    }

    void disconnect() override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        gripper_.reset();
    }

    bool isConnected() const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return gripper_ != nullptr;
    }

    bool homing() override
    {
        return call([](franka::Gripper& g) { return g.homing(); });
    }

    bool move(double width, double speed) override
    {
        return call([&](franka::Gripper& g) { return g.move(width, speed); });
    }

    bool grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer) override
    {
        return call([&](franka::Gripper& g) { return g.grasp(width, speed, force, epsilon_inner, epsilon_outer); });
    }

    bool stop() override
    {
        return call([](franka::Gripper& g) { return g.stop(); });
    }

    GripperSample readOnce() override
    {
        GripperSample sample;
        call([&](franka::Gripper& g) {
            const franka::GripperState state = g.readOnce();
            sample.width = state.width;
            sample.max_width = state.max_width;
            sample.is_grasped = state.is_grasped;
            sample.temperature = state.temperature;
            sample.time = state.time.toSec();
            return true;
        });
        return sample;
    }

private:
    // libfranka lets readOnce() and stop() run while a move or grasp blocks,
    // so each call holds its own reference rather than a lock.
    template <typename F>
    bool call(F&& f)
    {
        std::shared_ptr<franka::Gripper> gripper;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            gripper = gripper_;
        }
        if (!gripper)
            throw BackendNetworkError("not connected to the gripper");
        try
        {
            return f(*gripper);
        }
        catch (const franka::NetworkException& e)
        {
            disconnect();
            throw BackendNetworkError(e.what());
        }
        catch (const franka::Exception& e)
        {
            throw BackendControlError(e.what());
        }
    }

    const std::string robot_ip_;
    mutable std::mutex mutex_;
    std::shared_ptr<franka::Gripper> gripper_;
};

} // namespace

std::unique_ptr<Backend> makeFrankaBackend(const std::string& robot_ip, const Limits& limits, bool enforce_realtime)
{
    return std::make_unique<FrankaBackend>(robot_ip, limits, enforce_realtime);
}

std::unique_ptr<GripperBackend> makeFrankaGripperBackend(const std::string& robot_ip)
{
    return std::make_unique<FrankaGripperBackend>(robot_ip);
}

} // namespace csc376_franky
