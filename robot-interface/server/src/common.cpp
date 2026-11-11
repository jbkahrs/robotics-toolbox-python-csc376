#include "csc376_franky/common.hpp"

namespace csc376_franky
{

const char* robotModeName(int mode)
{
    switch (static_cast<RobotMode>(mode))
    {
    case RobotMode::Other: return "other";
    case RobotMode::Idle: return "idle";
    case RobotMode::Move: return "move";
    case RobotMode::Guiding: return "guiding";
    case RobotMode::Reflex: return "reflex";
    case RobotMode::UserStopped: return "user_stopped";
    case RobotMode::AutomaticErrorRecovery: return "automatic_error_recovery";
    }
    return "unknown";
}

const char* errorCodeName(ErrorCodes code)
{
    switch (code)
    {
    case ErrorCodes::Success: return "Success";
    case ErrorCodes::JointSizeNotSeven: return "JointSizeNotSeven";
    case ErrorCodes::JointDistancesAboveMaxJointDifference: return "JointDistancesAboveMaxJointDifference";
    case ErrorCodes::JointVelocitiesAboveMaximumAllowed: return "JointVelocitiesAboveMaximumAllowed";
    case ErrorCodes::DtIsNegative: return "DtIsNegative";
    case ErrorCodes::DtIsSmallerThanAllowed: return "DtIsSmallerThanAllowed";
    case ErrorCodes::Interrupted: return "Interrupted";
    case ErrorCodes::JointTrajectoryDoesNotStartAtCurrentPosition: return "JointTrajectoryDoesNotStartAtCurrentPosition";
    case ErrorCodes::TrajectorySizeLessThanTwo: return "TrajectorySizeLessThanTwo";
    case ErrorCodes::NonFiniteValue: return "NonFiniteValue";
    case ErrorCodes::JointLimitsExceeded: return "JointLimitsExceeded";
    case ErrorCodes::TrajectoryTooLong: return "TrajectoryTooLong";
    case ErrorCodes::DtIsLargerThanAllowed: return "DtIsLargerThanAllowed";
    case ErrorCodes::RobotBusy: return "RobotBusy";
    case ErrorCodes::RobotNotConnected: return "RobotNotConnected";
    case ErrorCodes::RobotNotReady: return "RobotNotReady";
    case ErrorCodes::ControlException: return "ControlException";
    case ErrorCodes::TrackingErrorTooLarge: return "TrackingErrorTooLarge";
    case ErrorCodes::ConnectionLost: return "ConnectionLost";
    case ErrorCodes::GripperNotConnected: return "GripperNotConnected";
    case ErrorCodes::GripperCommandFailed: return "GripperCommandFailed";
    case ErrorCodes::JointAccelerationsAboveMaximumAllowed: return "JointAccelerationsAboveMaximumAllowed";
    }
    return "Unknown";
}

} // namespace csc376_franky
