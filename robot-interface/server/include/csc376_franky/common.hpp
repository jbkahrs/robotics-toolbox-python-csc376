#pragma once

#include <array>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace csc376_franky
{

constexpr int kDof = 7;
using Vec7 = std::array<double, kDof>;
// A twist: linear velocity [m/s], then angular velocity [rad/s].
using Vec6 = std::array<double, 6>;
// A 6 x 7 Jacobian in column-major order, like franka::Model::zeroJacobian.
using Jacobian = std::array<double, 6 * kDof>;

// Franka Emika Panda (FER) joint limits, from the robot's datasheet.
constexpr Vec7 kJointPositionMin = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};
constexpr Vec7 kJointPositionMax = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973};
constexpr Vec7 kJointVelocityMax = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
constexpr Vec7 kJointAccelerationMax = {15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};

// franka::RobotMode, as reported in the robot state.
enum class RobotMode
{
    Other = 0,
    Idle = 1,
    Move = 2,
    Guiding = 3,
    Reflex = 4,
    UserStopped = 5,
    AutomaticErrorRecovery = 6,
};

const char* robotModeName(int mode);

// Codes -1 to -8 keep the values of the old csc376_bind_franky.ErrorCodes.
// csc376_franky/protocol.py mirrors this list; keep the two in step (-21 to
// -24 are used there, by the server and the client only).
enum class ErrorCodes
{
    Success = 1,
    JointSizeNotSeven = -1,
    JointDistancesAboveMaxJointDifference = -2,
    JointVelocitiesAboveMaximumAllowed = -3,
    DtIsNegative = -4,
    DtIsSmallerThanAllowed = -5,
    Interrupted = -6,
    JointTrajectoryDoesNotStartAtCurrentPosition = -7,
    TrajectorySizeLessThanTwo = -8,
    NonFiniteValue = -9,
    JointLimitsExceeded = -10,
    TrajectoryTooLong = -11,
    DtIsLargerThanAllowed = -12,
    RobotBusy = -13,
    RobotNotConnected = -14,
    RobotNotReady = -15,
    ControlException = -16,
    TrackingErrorTooLarge = -17,
    ConnectionLost = -18,
    GripperNotConnected = -19,
    GripperCommandFailed = -20,
    JointAccelerationsAboveMaximumAllowed = -25,
};

const char* errorCodeName(ErrorCodes code);

// JSON for the motion log's header lines: a quoted, escaped string; a number
// (null if not finite); an array of 7.
std::string jsonString(const std::string& text);
std::string jsonNumber(double value);
std::string jsonArray(const Vec7& values);

// What the miniPC lets a trajectory do. Set by whoever runs the server, never
// by the client: the defaults are the old controller's checks, made absolute.
struct Limits
{
    // A point may come no closer than this to a joint limit [rad].
    double joint_limit_margin = 0.01;
    // Fraction of kJointVelocityMax a trajectory may use.
    double velocity_scale = 0.3;
    // Fraction of kJointAccelerationMax the velocity may change by from one
    // segment to the next, per dt. Catches trajectories that zig-zag.
    double acceleration_scale = 1.0;
    // Largest change of one joint between consecutive points [rad]. Catches IK
    // solutions that flip to another branch.
    double max_joint_step = 0.025;
    // How far the first point may be from the measured joint positions [rad].
    double start_tolerance = 0.02;
    // Abort when a joint is this far from where it is commanded to be [rad].
    double tracking_error_limit = 0.05;
    double min_dt = 0.001;
    double max_dt = 1.0;
    double max_duration = 600.0;
    std::size_t max_points = 120000;
    // Time over which a gap between the measured start and the first point is
    // blended out, so the robot never jumps at the start [s].
    double start_blend_time = 0.25;
    // Time to slow down to rest along the path after stop() [s].
    double stop_time = 0.5;
    // After the last point, hold until every joint is slower than
    // settle_velocity [rad/s], for at most settle_timeout [s].
    double settle_velocity = 0.01;
    double settle_timeout = 1.0;
    // Joint impedance of the old controller [Nm/rad], [Nms/rad].
    Vec7 stiffness = {600.0, 900.0, 600.0, 300.0, 300.0, 200.0, 100.0};
    Vec7 damping = {50.0, 70.0, 70.0, 60.0, 30.0, 15.0, 10.0};
    // Joint impedance of the robot's own controller [Nm/rad], which follows
    // the joint position commands of move_home and the joint velocity
    // commands of velocity control (libfranka's setDefaultBehavior values).
    Vec7 joint_impedance = {3000.0, 3000.0, 3000.0, 2500.0, 2500.0, 2000.0, 2000.0};

    // Velocity control (servoing), where a client streams the twist the end
    // effector should follow. A twist faster than this is scaled down, keeping
    // its direction [m/s], [rad/s].
    double servo_linear_velocity = 0.1;
    double servo_angular_velocity = 0.5;
    // Time to reach those speeds from rest, and to slow down from them to rest [s].
    double servo_ramp_time = 0.3;
    // A twist older than this is replaced by zero, so the robot stops when its
    // client goes quiet [s].
    double servo_timeout = 0.25;
    // Joints slow down to rest this far inside their limits [rad].
    double servo_joint_limit_margin = 0.05;
    // Damping of the Jacobian's inverse, which bounds the joint velocities
    // near a singularity.
    double servo_damping = 0.02;
    // The end effector is not driven below this height above the robot's base [m].
    double servo_min_height = 0.02;
};

// Ends a trajectory early with a reason. Thrown from inside the control
// callback; libfranka stops the motion and passes it on.
struct TrajectoryAbort : public std::runtime_error
{
    TrajectoryAbort(ErrorCodes code, const std::string& message) : std::runtime_error(message), code(code) {}
    ErrorCodes code;
};

} // namespace csc376_franky
