#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "csc376_franky/common.hpp"

namespace csc376_franky
{

// The measured robot state the controller needs, copied out of
// franka::RobotState in the control callback (no allocation).
struct Measured
{
    Vec7 q{};
    Vec7 dq{};
    // Where the robot's own controller holds or commands the joints.
    Vec7 q_d{};
    Vec7 tau_J{};
    Vec7 tau_ext_hat_filtered{};
    std::array<double, 16> O_T_EE{};
    std::array<double, 6> O_F_ext_hat_K{};
    std::array<double, 2> elbow{};
    int robot_mode = static_cast<int>(RobotMode::Other);
    double time = 0.0;
    double control_command_success_rate = 0.0;
};

// What the controller asks for in one control cycle. The backend turns it
// into joint torques (joint impedance law) or, in simulation, into motion.
struct Command
{
    Vec7 q_d{};
    Vec7 dq_d{};
    bool finished = false;
};

// The connection to the robot dropped; it has to be opened again.
struct BackendNetworkError : public std::runtime_error
{
    using std::runtime_error::runtime_error;
};

// The robot refused or stopped a motion (reflex, user stop, ...). The
// connection stays usable.
struct BackendControlError : public std::runtime_error
{
    using std::runtime_error::runtime_error;
};

// A robot, real or simulated. Used from the controller's thread only, except
// errors(), which may be called from anywhere.
class Backend
{
public:
    using ReadCallback = std::function<bool(const Measured&)>;
    using ControlCallback = std::function<Command(const Measured&, double)>;

    virtual ~Backend() = default;

    virtual std::string name() const = 0;
    virtual std::string address() const = 0;
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;
    // Calls back with every robot state (1 kHz) until the callback returns false.
    virtual void read(const ReadCallback& callback) = 0;
    // Runs a 1 kHz control loop until a command has finished set. Exceptions
    // thrown by the callback stop the motion and are passed on unchanged.
    virtual void control(const ControlCallback& callback) = 0;
    // The same with libfranka's joint position interface: the robot's own
    // joint impedance controller follows q_d (dq_d is ignored).
    virtual void controlJointPositions(const ControlCallback& callback) = 0;
    virtual void recover() = 0;
    // Names of the robot's active errors and of the errors that ended the last motion.
    virtual void errors(std::vector<std::string>& current, std::vector<std::string>& last_motion) const = 0;
};

struct GripperSample
{
    double width = 0.0;
    double max_width = 0.0;
    bool is_grasped = false;
    double temperature = 0.0;
    double time = 0.0;
};

// A Franka Hand, real or simulated. homing/move/grasp block until done; stop()
// and readOnce() may be called from another thread while they run.
class GripperBackend
{
public:
    virtual ~GripperBackend() = default;

    virtual std::string name() const = 0;
    virtual void connect() = 0;
    virtual void disconnect() = 0;
    virtual bool isConnected() const = 0;
    virtual bool homing() = 0;
    virtual bool move(double width, double speed) = 0;
    virtual bool grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer) = 0;
    virtual bool stop() = 0;
    virtual GripperSample readOnce() = 0;
};

std::unique_ptr<Backend> makeSimBackend(const Vec7& q_start, double speed);
std::unique_ptr<GripperBackend> makeSimGripperBackend(double max_width, double speed);

#ifdef CSC376_HAVE_LIBFRANKA
std::unique_ptr<Backend> makeFrankaBackend(const std::string& robot_ip, const Limits& limits, bool enforce_realtime);
std::unique_ptr<GripperBackend> makeFrankaGripperBackend(const std::string& robot_ip);
#endif

} // namespace csc376_franky
