#include "calibration/types.hpp"

#include <cmath>

namespace calibration {

const char *sessionStateName(SessionState state) noexcept
{
    switch (state) {
        case SessionState::Inactive: return "inactive";
        case SessionState::Preparing: return "preparing";
        case SessionState::InitialPositioning: return "initial_positioning";
        case SessionState::AwaitingStartStop: return "awaiting_start_stop";
        case SessionState::StartReady: return "start_ready";
        case SessionState::Measuring: return "measuring";
        case SessionState::AwaitingEndStop: return "awaiting_end_stop";
        case SessionState::EndReady: return "end_ready";
        case SessionState::Review: return "review";
        case SessionState::Applying: return "applying";
        case SessionState::Saving: return "saving";
        case SessionState::Saved: return "saved";
        case SessionState::SaveFailed: return "save_failed";
        case SessionState::SaveUncertain: return "save_uncertain";
        case SessionState::Cancelling: return "cancelling";
        case SessionState::Cancelled: return "cancelled";
        case SessionState::Error: return "error";
    }
    return "unknown";
}

const char *sessionErrorName(SessionError error) noexcept
{
    switch (error) {
        case SessionError::None: return "none";
        case SessionError::ControlBusy: return "control_busy";
        case SessionError::HoldUnavailable: return "hold_unavailable";
        case SessionError::DirectionViolation: return "direction_violation";
        case SessionError::ClientLeaseExpired: return "motion_lease_expired";
        case SessionError::StopUnconfirmed: return "stop_unconfirmed";
        case SessionError::ReferenceConfirmTimeout: return "reference_confirm_timeout";
        case SessionError::SaveFailed: return "save_failed";
        case SessionError::SaveUncertain: return "save_uncertain";
        case SessionError::MeasurementInvalid: return "measurement_invalid";
        case SessionError::SessionExpired: return "session_expired";
    }
    return "unknown";
}

const char *invalidReasonName(InvalidReason reason) noexcept
{
    switch (reason) {
        case InvalidReason::Ok: return "ok";
        case InvalidReason::ZeroDelta: return "zero_delta";
        case InvalidReason::DirectionMismatch: return "direction_mismatch";
        case InvalidReason::PositionUntrusted: return "position_untrusted";
        case InvalidReason::Fault: return "fault";
        case InvalidReason::Reversal: return "reversal";
        case InvalidReason::EmergencyStop: return "emergency_stop";
        case InvalidReason::StopUnconfirmed: return "stop_unconfirmed";
        case InvalidReason::EpochChanged: return "epoch_changed";
        case InvalidReason::LeaseExpired: return "lease_expired";
        case InvalidReason::HoldLost: return "hold_lost";
        case InvalidReason::HoldUnavailable: return "hold_unavailable";
        case InvalidReason::ClientAbort: return "client_abort";
        case InvalidReason::InvalidResult: return "invalid_result";
    }
    return "invalid_result";
}

const char *repeatabilityName(Repeatability value) noexcept
{
    switch (value) {
        case Repeatability::Insufficient: return "insufficient";
        case Repeatability::Good: return "good";
        case Repeatability::Warn: return "warn";
        case Repeatability::Invalid: return "invalid";
    }
    return "invalid";
}

const char *persistenceName(Persistence value) noexcept
{
    switch (value) {
        case Persistence::Absent: return "absent";
        case Persistence::Verified: return "verified";
        case Persistence::Failed: return "failed";
        case Persistence::Unconfirmed: return "unconfirmed";
    }
    return "unconfirmed";
}

const char *sourceName(AxisScaleSource value) noexcept
{
    switch (value) {
        case AxisScaleSource::ConfigNominal: return "config_nominal";
        case AxisScaleSource::SavedNominal: return "saved_nominal";
        case AxisScaleSource::User: return "user";
    }
    return "config_nominal";
}

bool validParams(const CalibrationParams &params) noexcept
{
    if (params.min_revolutions == 0 || params.max_revolutions < params.min_revolutions) {
        return false;
    }
    if (params.max_revolutions > kMaxRevolutions) {
        return false;
    }
    if (params.min_measurements < 2 || params.max_measurements < params.min_measurements ||
        params.max_measurements > kMaxMeasurements) {
        return false;
    }
    if (!std::isfinite(params.min_degrees_per_half_step) ||
        !std::isfinite(params.max_degrees_per_half_step) ||
        params.min_degrees_per_half_step <= 0.0 ||
        params.max_degrees_per_half_step <= params.min_degrees_per_half_step) {
        return false;
    }
    if (params.max_spread_ppm == 0) {
        return false;
    }
    for (std::uint32_t nominal : params.nominal_half_steps_per_revolution) {
        if (nominal == 0) {
            return false;
        }
    }
    return true;
}

bool validConfig(const SessionConfig &config, const CalibrationParams &params) noexcept
{
    if (!validParams(params)) {
        return false;
    }
    if (config.revolutions < params.min_revolutions ||
        config.revolutions > params.max_revolutions) {
        return false;
    }
    if (config.repeats < params.min_measurements ||
        config.repeats > params.max_measurements) {
        return false;
    }
    return config.direction == motion::Direction::Forward ||
           config.direction == motion::Direction::Backward;
}

}  // namespace calibration
