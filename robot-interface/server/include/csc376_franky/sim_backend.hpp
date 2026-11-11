#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "csc376_franky/backend.hpp"

namespace csc376_franky
{

// A stand-in for the robot, for testing the server and clients without one.
// The arm follows the commanded joint state as a critically damped second
// order system; O_T_EE is the Panda's forward kinematics with the Franka Hand
// TCP, as Desk sets it up. speed > 1 runs faster than real time.
class SimBackend : public Backend
{
public:
    SimBackend(const Vec7& q_start, double speed);

    std::string name() const override { return "sim"; }
    std::string address() const override { return "simulation"; }
    void connect() override;
    void disconnect() override;
    bool isConnected() const override;
    void read(const ReadCallback& callback) override;
    void control(const ControlCallback& callback) override;
    void controlJointPositions(const ControlCallback& callback) override;
    void recover() override;
    void errors(std::vector<std::string>& current, std::vector<std::string>& last_motion) const override;

    // Test hooks, callable from any thread.
    // Moves the arm, as if it was guided by hand.
    void setJointPositions(const Vec7& q);
    void setRobotMode(RobotMode mode);
    // The next control cycle ends in a cartesian reflex.
    void injectReflex();
    // Pushes a joint away from where it is, e.g. past the tracking error limit.
    void injectDisturbance(int joint, double offset);
    // The next cycle loses the connection.
    void dropConnection();
    // connect() fails with reason until called again with an empty reason.
    void refuseConnections(const std::string& reason);

private:
    Measured measuredLocked() const;
    // The control loop of control() and controlJointPositions().
    void run(const ControlCallback& callback, bool joint_positions);

    mutable std::mutex mutex_;
    double speed_;
    bool connected_ = false;
    std::string refuse_reason_;
    Vec7 q_{};
    Vec7 dq_{};
    double time_ = 0.0;
    int mode_ = static_cast<int>(RobotMode::Idle);
    bool reflex_pending_ = false;
    bool drop_pending_ = false;
    std::vector<std::string> current_errors_;
    std::vector<std::string> last_motion_errors_;
};

// A stand-in for the Franka Hand. Fingers move at the commanded speed; a
// grasp closes on the object set with setObjectWidth, if there is one.
class SimGripperBackend : public GripperBackend
{
public:
    SimGripperBackend(double max_width, double speed);

    std::string name() const override { return "sim"; }
    void connect() override;
    void disconnect() override;
    bool isConnected() const override;
    bool homing() override;
    bool move(double width, double speed) override;
    bool grasp(double width, double speed, double force, double epsilon_inner, double epsilon_outer) override;
    bool stop() override;
    GripperSample readOnce() override;

    // Width of the object between the fingers; negative for none.
    void setObjectWidth(double width);

private:
    // Moves the fingers towards target, stopping at the object if stop_at_object.
    bool moveTo(double target, double speed, bool stop_at_object);

    mutable std::mutex mutex_;
    double speed_;
    double max_width_;
    double width_;
    double object_width_ = -1.0;
    bool is_grasped_ = false;
    bool connected_ = false;
    bool stop_requested_ = false;
    double time_ = 0.0;
};

} // namespace csc376_franky
