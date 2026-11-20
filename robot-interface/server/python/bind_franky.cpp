#include <optional>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "csc376_franky/controller.hpp"
#include "csc376_franky/gripper_controller.hpp"
#include "csc376_franky/motion_generator.hpp"
#include "csc376_franky/sim_backend.hpp"

namespace py = pybind11;
using namespace pybind11::literals; // to bring in the '_a' literal
using namespace csc376_franky;

namespace
{

using Array = py::array_t<double, py::array::c_style | py::array::forcecast>;

// The toolbox's ready pose (rtb.models.Panda().qr), where the simulated robot starts.
constexpr Vec7 kReadyPose = {0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.7853981633974483};

// Copies the rows of an N x 7 array; false (with a message) for any other shape.
bool toPoints(const Array& array, std::vector<Vec7>& points, std::string& message)
{
    if (array.ndim() != 2 || array.shape(1) != kDof)
    {
        message = "Trajectory must be an N x 7 array of joint positions";
        if (array.ndim() == 2)
            message += " (it has " + std::to_string(array.shape(1)) + " columns)";
        return false;
    }
    const auto rows = array.unchecked<2>();
    points.resize(static_cast<std::size_t>(rows.shape(0)));
    for (py::ssize_t i = 0; i < rows.shape(0); ++i)
        for (int j = 0; j < kDof; ++j)
            points[static_cast<std::size_t>(i)][j] = rows(i, j);
    return true;
}

const char* statusName(Controller::Status status)
{
    switch (status)
    {
    case Controller::Status::Disconnected: return "disconnected";
    case Controller::Status::Connecting: return "connecting";
    case Controller::Status::Idle: return "idle";
    case Controller::Status::Executing: return "executing";
    case Controller::Status::Recovering: return "recovering";
    }
    return "unknown";
}

const char* phaseName(TrajectoryFollower::Phase phase)
{
    switch (phase)
    {
    case TrajectoryFollower::Phase::Running: return "running";
    case TrajectoryFollower::Phase::Stopping: return "stopping";
    case TrajectoryFollower::Phase::Settling: return "settling";
    case TrajectoryFollower::Phase::Done: return "done";
    }
    return "unknown";
}

const char* motionName(Controller::Motion motion)
{
    switch (motion)
    {
    case Controller::Motion::None: return "none";
    case Controller::Motion::Trajectory: return "trajectory";
    case Controller::Motion::JointMotion: return "joint_motion";
    case Controller::Motion::Servo: return "servo";
    }
    return "unknown";
}

const char* followerPhaseName(TwistFollower::Phase phase)
{
    switch (phase)
    {
    case TwistFollower::Phase::Running: return "running";
    case TwistFollower::Phase::Stopping: return "stopping";
    case TwistFollower::Phase::Done: return "done";
    }
    return "unknown";
}

// The names csc376_franky/protocol.py gives the frames of a twist.
TwistFrame frameFromName(const std::string& name)
{
    if (name == "base")
        return TwistFrame::Base;
    if (name == "end_effector")
        return TwistFrame::EndEffector;
    throw py::value_error("unknown twist frame: " + name);
}

py::list limitedNames(unsigned limited)
{
    py::list names;
    for (const auto bit : {TwistFollower::Timeout, TwistFollower::Speed, TwistFollower::Height,
                           TwistFollower::JointVelocity, TwistFollower::JointPosition, TwistFollower::Singularity})
    {
        if ((limited & bit) != 0)
            names.append(limitedName(bit));
    }
    return names;
}

// A 6 x 7 array, as numpy has it, from a Jacobian (column-major) and back.
Array jacobianArray(const Jacobian& J)
{
    Array array({6, kDof});
    auto rows = array.mutable_unchecked<2>();
    for (int r = 0; r < 6; ++r)
        for (int j = 0; j < kDof; ++j)
            rows(r, j) = J[j * 6 + r];
    return array;
}

Jacobian toJacobian(const Array& array)
{
    if (array.ndim() != 2 || array.shape(0) != 6 || array.shape(1) != kDof)
        throw py::value_error("The Jacobian must be a 6 x 7 array");
    const auto rows = array.unchecked<2>();
    Jacobian J{};
    for (int r = 0; r < 6; ++r)
        for (int j = 0; j < kDof; ++j)
            J[j * 6 + r] = rows(r, j);
    return J;
}

py::object textOrNone(const std::string& text)
{
    return text.empty() ? py::object(py::none()) : py::object(py::str(text));
}

py::dict stateDict(const Measured& m)
{
    py::dict d;
    d["q"] = m.q;
    d["dq"] = m.dq;
    d["tau_J"] = m.tau_J;
    d["tau_ext_hat_filtered"] = m.tau_ext_hat_filtered;
    d["O_T_EE"] = m.O_T_EE;
    d["O_F_ext_hat_K"] = m.O_F_ext_hat_K;
    d["elbow"] = m.elbow;
    d["robot_mode"] = robotModeName(m.robot_mode);
    d["time"] = m.time;
    d["control_command_success_rate"] = m.control_command_success_rate;
    return d;
}

py::dict snapshotDict(const Controller::Snapshot& s)
{
    py::dict d;
    d["status"] = statusName(s.status);
    d["connected"] = s.connected;
    d["connection_error"] = textOrNone(s.connection_error);
    d["seq"] = s.seq;
    d["age"] = s.age;
    d["state"] = s.has_state ? py::object(stateDict(s.state)) : py::object(py::none());
    d["current_errors"] = s.current_errors;
    d["last_motion_errors"] = s.last_motion_errors;
    if (s.active_id != 0)
    {
        py::dict t;
        t["id"] = s.active_id;
        t["kind"] = motionName(s.motion);
        t["index"] = s.index;
        t["length"] = s.length;
        t["time"] = s.time;
        t["duration"] = s.duration;
        t["stopping"] = s.stopping;
        t["settling"] = s.settling;
        t["q_d"] = s.q_d;
        t["dq_d"] = s.dq_d;
        if (s.motion == Controller::Motion::Servo)
        {
            t["twist"] = s.twist;
            t["limited"] = limitedNames(s.limited);
        }
        d["trajectory"] = t;
    }
    else
    {
        d["trajectory"] = py::none();
    }
    py::list results;
    for (const auto& r : s.results)
        results.append(py::make_tuple(r.id, static_cast<int>(r.code), r.message, r.log_file));
    d["results"] = results;
    return d;
}

py::tuple submissionTuple(const Controller::Submission& s)
{
    return py::make_tuple(static_cast<int>(s.code), s.message, s.id);
}

py::tuple resultTuple(const GripperController::Result& r)
{
    return py::make_tuple(static_cast<int>(r.code), r.success, r.message);
}

SimBackend& simBackend(Controller& controller)
{
    auto* sim = dynamic_cast<SimBackend*>(&controller.backend());
    if (sim == nullptr)
        throw std::runtime_error("only available with the simulated robot");
    return *sim;
}

SimGripperBackend& simGripperBackend(GripperController& gripper)
{
    auto* sim = dynamic_cast<SimGripperBackend*>(&gripper.backend());
    if (sim == nullptr)
        throw std::runtime_error("only available with the simulated gripper");
    return *sim;
}

RobotMode modeFromName(const std::string& name)
{
    for (int mode = 0; mode <= static_cast<int>(RobotMode::AutomaticErrorRecovery); ++mode)
        if (name == robotModeName(mode))
            return static_cast<RobotMode>(mode);
    throw py::value_error("unknown robot mode: " + name);
}

} // namespace

PYBIND11_MODULE(_control, m)
{
    m.doc() = "Real-time trajectory execution on the station's Panda, for the CSC376 robot server";

    py::enum_<ErrorCodes>(m, "ErrorCodes")
        .value("Success", ErrorCodes::Success)
        .value("JointSizeNotSeven", ErrorCodes::JointSizeNotSeven)
        .value("JointDistancesAboveMaxJointDifference", ErrorCodes::JointDistancesAboveMaxJointDifference)
        .value("JointVelocitiesAboveMaximumAllowed", ErrorCodes::JointVelocitiesAboveMaximumAllowed)
        .value("DtIsNegative", ErrorCodes::DtIsNegative)
        .value("DtIsSmallerThanAllowed", ErrorCodes::DtIsSmallerThanAllowed)
        .value("Interrupted", ErrorCodes::Interrupted)
        .value("JointTrajectoryDoesNotStartAtCurrentPosition", ErrorCodes::JointTrajectoryDoesNotStartAtCurrentPosition)
        .value("TrajectorySizeLessThanTwo", ErrorCodes::TrajectorySizeLessThanTwo)
        .value("NonFiniteValue", ErrorCodes::NonFiniteValue)
        .value("JointLimitsExceeded", ErrorCodes::JointLimitsExceeded)
        .value("TrajectoryTooLong", ErrorCodes::TrajectoryTooLong)
        .value("DtIsLargerThanAllowed", ErrorCodes::DtIsLargerThanAllowed)
        .value("RobotBusy", ErrorCodes::RobotBusy)
        .value("RobotNotConnected", ErrorCodes::RobotNotConnected)
        .value("RobotNotReady", ErrorCodes::RobotNotReady)
        .value("ControlException", ErrorCodes::ControlException)
        .value("TrackingErrorTooLarge", ErrorCodes::TrackingErrorTooLarge)
        .value("ConnectionLost", ErrorCodes::ConnectionLost)
        .value("GripperNotConnected", ErrorCodes::GripperNotConnected)
        .value("GripperCommandFailed", ErrorCodes::GripperCommandFailed)
        .value("JointAccelerationsAboveMaximumAllowed", ErrorCodes::JointAccelerationsAboveMaximumAllowed);

    py::class_<Limits>(m, "Limits")
        .def(py::init<>())
        .def_readwrite("joint_limit_margin", &Limits::joint_limit_margin)
        .def_readwrite("velocity_scale", &Limits::velocity_scale)
        .def_readwrite("acceleration_scale", &Limits::acceleration_scale)
        .def_readwrite("max_joint_step", &Limits::max_joint_step)
        .def_readwrite("start_tolerance", &Limits::start_tolerance)
        .def_readwrite("tracking_error_limit", &Limits::tracking_error_limit)
        .def_readwrite("min_dt", &Limits::min_dt)
        .def_readwrite("max_dt", &Limits::max_dt)
        .def_readwrite("max_duration", &Limits::max_duration)
        .def_readwrite("max_points", &Limits::max_points)
        .def_readwrite("start_blend_time", &Limits::start_blend_time)
        .def_readwrite("stop_time", &Limits::stop_time)
        .def_readwrite("settle_velocity", &Limits::settle_velocity)
        .def_readwrite("settle_timeout", &Limits::settle_timeout)
        .def_readwrite("stiffness", &Limits::stiffness)
        .def_readwrite("damping", &Limits::damping)
        .def_readwrite("joint_impedance", &Limits::joint_impedance)
        .def_readwrite("servo_linear_velocity", &Limits::servo_linear_velocity)
        .def_readwrite("servo_angular_velocity", &Limits::servo_angular_velocity)
        .def_readwrite("servo_ramp_time", &Limits::servo_ramp_time)
        .def_readwrite("servo_timeout", &Limits::servo_timeout)
        .def_readwrite("servo_joint_limit_margin", &Limits::servo_joint_limit_margin)
        .def_readwrite("servo_damping", &Limits::servo_damping)
        .def_readwrite("servo_min_height", &Limits::servo_min_height);

    m.def(
        "validate_trajectory",
        [](const Array& trajectory, double dt, std::optional<Vec7> q_current, const Limits& limits) -> py::tuple {
            std::vector<Vec7> points;
            std::string message;
            if (!toPoints(trajectory, points, message))
                return py::make_tuple(static_cast<int>(ErrorCodes::JointSizeNotSeven), message);
            const ValidationResult result = validateTrajectory(points, dt, q_current ? &*q_current : nullptr, limits);
            return py::make_tuple(static_cast<int>(result.code), result.message);
        },
        "trajectory"_a, "dt"_a, "q_current"_a = py::none(), "limits"_a = Limits(),
        "Check a trajectory; returns (error code, message)");

    py::class_<TrajectoryFollower>(m, "TrajectoryFollower")
        .def(py::init([](const Array& trajectory, double dt, const Limits& limits) {
                 std::vector<Vec7> points;
                 std::string message;
                 if (!toPoints(trajectory, points, message))
                     throw py::value_error(message);
                 return std::make_unique<TrajectoryFollower>(std::move(points), dt, limits);
             }),
             "trajectory"_a, "dt"_a, "limits"_a = Limits())
        .def("step", &TrajectoryFollower::step, "period"_a, "q"_a, "dq"_a, "stop_requested"_a = false)
        .def_property_readonly("q_d", &TrajectoryFollower::qDesired)
        .def_property_readonly("dq_d", &TrajectoryFollower::dqDesired)
        .def_property_readonly("phase", [](const TrajectoryFollower& f) { return phaseName(f.phase()); })
        .def_property_readonly("interrupted", &TrajectoryFollower::interrupted)
        .def_property_readonly("index", &TrajectoryFollower::index)
        .def_property_readonly("time", &TrajectoryFollower::time)
        .def_property_readonly("duration", &TrajectoryFollower::duration);

    py::class_<TwistFollower>(m, "TwistFollower")
        .def(py::init<const Limits&>(), "limits"_a = Limits())
        .def(
            "set_target",
            [](TwistFollower& f, const Vec6& twist, const std::string& frame) {
                f.setTarget(twist, frameFromName(frame));
            },
            "twist"_a, "frame"_a = "end_effector")
        .def(
            "step",
            [](TwistFollower& f, double period, const Vec7& q, const Vec7& dq, const std::array<double, 16>& O_T_EE,
               const Array& jacobian, bool stop_requested) {
                Measured measured;
                measured.q = q;
                measured.q_d = q;
                measured.dq = dq;
                measured.O_T_EE = O_T_EE;
                f.step(period, measured, toJacobian(jacobian), stop_requested, pandaJacobian);
            },
            "period"_a, "q"_a, "dq"_a, "O_T_EE"_a, "jacobian"_a, "stop_requested"_a = false)
        .def_property_readonly("dq_c", &TwistFollower::dqCommand)
        .def_property_readonly("twist", &TwistFollower::twist)
        .def_property_readonly("phase", [](const TwistFollower& f) { return followerPhaseName(f.phase()); })
        .def_property_readonly("limited", [](const TwistFollower& f) { return limitedNames(f.limited()); });

    m.def("sim_pose", &pandaPose, "q"_a, "The simulated robot's end-effector pose in the base frame, like O_T_EE");
    m.def(
        "sim_jacobian", [](const Vec7& q) { return jacobianArray(pandaJacobian(q)); }, "q"_a,
        "The simulated robot's end-effector Jacobian in the base frame, 6 x 7");

    py::class_<JointMotionGenerator>(m, "JointMotionGenerator")
        .def(py::init<const Vec7&, const Vec7&, double>(), "q_start"_a, "q_goal"_a, "speed_factor"_a)
        .def(
            "positions",
            [](const JointMotionGenerator& g, double t) {
                Vec7 q_d{};
                const bool finished = g.positions(t, q_d);
                return py::make_tuple(finished, q_d);
            },
            "t"_a, "(finished, joint positions) t seconds after the start")
        .def_property_readonly("duration", &JointMotionGenerator::duration);

    py::class_<MotionLog, std::shared_ptr<MotionLog>>(
        m, "MotionLog",
        "Records every control cycle of every motion to a CSV file of its own, for tuning the controller")
        .def(py::init([](const std::string& directory, std::uintmax_t max_bytes, const std::string& preamble) {
                 return std::make_shared<MotionLog>(directory, max_bytes, preamble);
             }),
             "directory"_a, "max_bytes"_a, "preamble"_a = "",
             "Log into directory, removing the oldest files once they take more than max_bytes together; "
             "preamble is '# ' lines for the head of every file")
        .def_property_readonly("directory", [](const MotionLog& log) { return log.directory().string(); })
        .def_property_readonly(
            "error", [](const MotionLog& log) { return textOrNone(log.error()); },
            "What last went wrong writing, or None")
        .def_property_readonly("files", &MotionLog::files, "Files finished since the start")
        .def_property_readonly("dropped", &MotionLog::dropped, "Samples dropped since the start")
        .def("shutdown", &MotionLog::shutdown, py::call_guard<py::gil_scoped_release>(),
             "Write what is queued and end the log's thread");

    py::class_<Controller>(m, "Controller")
        .def_static(
            "sim",
            [](const Limits& limits, const Vec7& q_start, double speed, std::shared_ptr<MotionLog> log) {
                return std::make_unique<Controller>(makeSimBackend(q_start, speed), limits, std::move(log));
            },
            "limits"_a = Limits(), "q_start"_a = kReadyPose, "speed"_a = 1.0, "log"_a = py::none(),
            "A controller for a simulated robot; speed > 1 runs faster than real time. With a MotionLog, "
            "every motion is recorded there")
#ifdef CSC376_HAVE_LIBFRANKA
        .def_static(
            "franka",
            [](const std::string& robot_ip, const Limits& limits, bool enforce_realtime,
               std::shared_ptr<MotionLog> log) {
                return std::make_unique<Controller>(makeFrankaBackend(robot_ip, limits, enforce_realtime), limits,
                                                    std::move(log));
            },
            "robot_ip"_a, "limits"_a = Limits(), "enforce_realtime"_a = true, "log"_a = py::none(),
            "A controller for the robot at robot_ip (FCI). With a MotionLog, every motion is recorded there")
#endif
        .def_property_readonly("backend", [](Controller& c) { return c.backend().name(); })
        .def_property_readonly("address", [](Controller& c) { return c.backend().address(); })
        .def_property_readonly("limits", [](const Controller& c) { return c.limits(); })
        .def("set_connection_wanted", &Controller::setConnectionWanted, "wanted"_a,
             py::call_guard<py::gil_scoped_release>())
        .def(
            "validate",
            [](const Controller& c, const Array& trajectory, double dt) -> py::tuple {
                std::vector<Vec7> points;
                std::string message;
                if (!toPoints(trajectory, points, message))
                    return py::make_tuple(static_cast<int>(ErrorCodes::JointSizeNotSeven), message);
                ValidationResult result;
                {
                    py::gil_scoped_release release;
                    result = c.validate(points, dt);
                }
                return py::make_tuple(static_cast<int>(result.code), result.message);
            },
            "trajectory"_a, "dt"_a)
        .def(
            "execute",
            [](Controller& c, const Array& trajectory, double dt) -> py::tuple {
                std::vector<Vec7> points;
                std::string message;
                if (!toPoints(trajectory, points, message))
                    return py::make_tuple(static_cast<int>(ErrorCodes::JointSizeNotSeven), message, 0);
                Controller::Submission submission;
                {
                    py::gil_scoped_release release;
                    submission = c.execute(std::move(points), dt);
                }
                return submissionTuple(submission);
            },
            "trajectory"_a, "dt"_a, "Start a trajectory; returns (error code, message, id)")
        .def(
            "move_to_joint_positions",
            [](Controller& c, const Vec7& q_goal, double speed_factor) {
                Controller::Submission submission;
                {
                    py::gil_scoped_release release;
                    submission = c.moveToJointPositions(q_goal, speed_factor);
                }
                return submissionTuple(submission);
            },
            "q_goal"_a, "speed_factor"_a,
            "Start a move with libfranka's example MotionGenerator; returns (error code, message, id)")
        .def(
            "start_servo",
            [](Controller& c) {
                Controller::Submission submission;
                {
                    py::gil_scoped_release release;
                    submission = c.startServo();
                }
                return submissionTuple(submission);
            },
            "Start velocity control; returns (error code, message, id)")
        .def(
            "set_servo_twist",
            [](Controller& c, std::uint64_t id, const Vec6& twist, const std::string& frame) {
                const TwistFrame twist_frame = frameFromName(frame);
                ValidationResult result;
                {
                    py::gil_scoped_release release;
                    result = c.setServoTwist(id, twist, twist_frame);
                }
                return py::make_tuple(static_cast<int>(result.code), result.message);
            },
            "id"_a, "twist"_a, "frame"_a = "end_effector",
            "The twist for the velocity control with this id to follow; returns (error code, message)")
        .def("end_servo", &Controller::endServo, "id"_a, py::call_guard<py::gil_scoped_release>(),
             "Slow the velocity control with this id down to rest and end it")
        .def(
            "recover",
            [](Controller& c) {
                Controller::Submission submission;
                {
                    py::gil_scoped_release release;
                    submission = c.recover();
                }
                return submissionTuple(submission);
            },
            "Start an error recovery; returns (error code, message, id)")
        .def("stop", &Controller::stop, py::call_guard<py::gil_scoped_release>())
        .def("snapshot",
             [](const Controller& c) {
                 Controller::Snapshot snapshot;
                 {
                     py::gil_scoped_release release;
                     snapshot = c.snapshot();
                 }
                 return snapshotDict(snapshot);
             })
        .def("shutdown", &Controller::shutdown, py::call_guard<py::gil_scoped_release>())
        // Fault injection, simulated robot only.
        .def(
            "sim_set_joint_positions", [](Controller& c, const Vec7& q) { simBackend(c).setJointPositions(q); }, "q"_a)
        .def(
            "sim_set_robot_mode",
            [](Controller& c, const std::string& mode) { simBackend(c).setRobotMode(modeFromName(mode)); }, "mode"_a)
        .def("sim_inject_reflex", [](Controller& c) { simBackend(c).injectReflex(); })
        .def(
            "sim_inject_disturbance",
            [](Controller& c, int joint, double offset) { simBackend(c).injectDisturbance(joint, offset); }, "joint"_a,
            "offset"_a)
        .def("sim_drop_connection", [](Controller& c) { simBackend(c).dropConnection(); })
        .def(
            "sim_refuse_connections",
            [](Controller& c, const std::string& reason) { simBackend(c).refuseConnections(reason); }, "reason"_a);

    py::class_<GripperController>(m, "GripperController")
        .def_static(
            "sim",
            [](double max_width, double speed) {
                return std::make_unique<GripperController>(makeSimGripperBackend(max_width, speed));
            },
            "max_width"_a = 0.08, "speed"_a = 1.0)
#ifdef CSC376_HAVE_LIBFRANKA
        .def_static(
            "franka",
            [](const std::string& robot_ip) {
                return std::make_unique<GripperController>(makeFrankaGripperBackend(robot_ip));
            },
            "robot_ip"_a)
#endif
        .def_property_readonly("backend", [](GripperController& g) { return g.backend().name(); })
        .def("set_connection_wanted", &GripperController::setConnectionWanted, "wanted"_a,
             py::call_guard<py::gil_scoped_release>())
        .def("homing",
             [](GripperController& g) {
                 GripperController::Result result;
                 {
                     py::gil_scoped_release release;
                     result = g.homing();
                 }
                 return resultTuple(result);
             })
        .def(
            "move",
            [](GripperController& g, double width, double speed) {
                GripperController::Result result;
                {
                    py::gil_scoped_release release;
                    result = g.move(width, speed);
                }
                return resultTuple(result);
            },
            "width"_a, "speed"_a)
        .def(
            "grasp",
            [](GripperController& g, double width, double speed, double force, double epsilon_inner,
               double epsilon_outer) {
                GripperController::Result result;
                {
                    py::gil_scoped_release release;
                    result = g.grasp(width, speed, force, epsilon_inner, epsilon_outer);
                }
                return resultTuple(result);
            },
            "width"_a, "speed"_a, "force"_a, "epsilon_inner"_a = 0.005, "epsilon_outer"_a = 0.005)
        .def("stop",
             [](GripperController& g) {
                 GripperController::Result result;
                 {
                     py::gil_scoped_release release;
                     result = g.stop();
                 }
                 return resultTuple(result);
             })
        .def("snapshot",
             [](const GripperController& g) {
                 GripperController::Snapshot s;
                 {
                     py::gil_scoped_release release;
                     s = g.snapshot();
                 }
                 py::dict d;
                 d["connected"] = s.connected;
                 d["connection_error"] = textOrNone(s.connection_error);
                 d["age"] = s.age;
                 d["busy"] = s.busy;
                 if (s.has_state)
                 {
                     py::dict state;
                     state["width"] = s.state.width;
                     state["max_width"] = s.state.max_width;
                     state["is_grasped"] = s.state.is_grasped;
                     state["temperature"] = s.state.temperature;
                     state["time"] = s.state.time;
                     d["state"] = state;
                 }
                 else
                 {
                     d["state"] = py::none();
                 }
                 return d;
             })
        .def("shutdown", &GripperController::shutdown, py::call_guard<py::gil_scoped_release>())
        .def(
            "sim_set_object_width",
            [](GripperController& g, double width) { simGripperBackend(g).setObjectWidth(width); }, "width"_a);

#ifdef CSC376_HAVE_LIBFRANKA
    m.attr("HAS_LIBFRANKA") = true;
    m.attr("LIBFRANKA_VERSION") = CSC376_LIBFRANKA_VERSION;
#else
    m.attr("HAS_LIBFRANKA") = false;
    m.attr("LIBFRANKA_VERSION") = py::none();
#endif
    m.attr("READY_POSE") = kReadyPose;
}
