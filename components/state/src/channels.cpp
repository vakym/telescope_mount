/**
 * @file channels.cpp
 * @brief Текстовые имена словаря состояния для диагностики.
 */

#include "state/channels.hpp"

namespace state {

const char *sourceName(SourceId source) noexcept
{
    switch (source) {
        case SourceId::None:
            return "none";
        case SourceId::Joystick:
            return "joystick";
        case SourceId::Browser:
            return "browser";
        case SourceId::Automation:
            return "automation";
        case SourceId::Calibration:
            return "calibration";
        case SourceId::Count:
            break;
    }
    return "unknown";
}

const char *phaseName(ControlPhase phase) noexcept
{
    switch (phase) {
        case ControlPhase::Disarmed:
            return "disarmed";
        case ControlPhase::WaitingForNeutral:
            return "waiting_neutral";
        case ControlPhase::Enabling:
            return "enabling";
        case ControlPhase::Active:
            return "active";
        case ControlPhase::Stopping:
            return "stopping";
    }
    return "unknown";
}

const char *latchName(LatchCode latch) noexcept
{
    switch (latch) {
        case LatchCode::None:
            return "none";
        case LatchCode::MotorFault:
            return "motor_fault";
        case LatchCode::DeliveryFailed:
            return "delivery_failed";
        case LatchCode::StopUnconfirmed:
            return "stop_unconfirmed";
        case LatchCode::ConfigInvalid:
            return "config_invalid";
    }
    return "unknown";
}

const char *axisStateName(motion::AxisState state) noexcept
{
    switch (state) {
        case motion::AxisState::Disabled:
            return "disabled";
        case motion::AxisState::Idle:
            return "idle";
        case motion::AxisState::Accelerating:
            return "accelerating";
        case motion::AxisState::Cruising:
            return "cruising";
        case motion::AxisState::Decelerating:
            return "decelerating";
        case motion::AxisState::Holding:
            return "holding";
        case motion::AxisState::Released:
            return "released";
        case motion::AxisState::Fault:
            return "fault";
    }
    return "unknown";
}

const char *faultReasonName(motion::FaultReason reason) noexcept
{
    switch (reason) {
        case motion::FaultReason::DeadlineMiss:
            return "deadline_miss";
        case motion::FaultReason::ArmFailure:
            return "arm_failure";
        case motion::FaultReason::WatchdogTimeout:
            return "watchdog_timeout";
        case motion::FaultReason::InvalidState:
            return "invalid_state";
    }
    return "unknown";
}

const char *denyName(DenyReason reason) noexcept
{
    switch (reason) {
        case DenyReason::None:
            return "none";
        case DenyReason::NotCalibrated:
            return "not_calibrated";
        case DenyReason::NoSample:
            return "no_sample";
        case DenyReason::InvalidSample:
            return "invalid_sample";
        case DenyReason::ReadFault:
            return "read_fault";
        case DenyReason::StaleInput:
            return "stale_input";
        case DenyReason::InvalidConfig:
            return "invalid_config";
        case DenyReason::NotOwner:
            return "not_owner";
        case DenyReason::OldSession:
            return "old_session";
        case DenyReason::RepeatedSequence:
            return "repeated_sequence";
        case DenyReason::MotorFault:
            return "motor_fault";
        case DenyReason::DeliveryFailed:
            return "delivery_failed";
        case DenyReason::StopInProgress:
            return "stop_in_progress";
        case DenyReason::NotReady:
            return "not_ready";
        case DenyReason::Busy:
            return "busy";
        case DenyReason::NeutralRequired:
            return "neutral_required";
    }
    return "unknown";
}

}  // namespace state
