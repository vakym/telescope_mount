/**
 * @file types.hpp
 * @brief Чистые типы калибровки масштаба механики.
 *
 * Компонент не зависит от ESP-IDF, FreeRTOS и NVS: только типы motion и
 * стандартная библиотека C++17. Все временные величины передаются явно.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "motion/types.hpp"
#include "state/observation.hpp"

namespace calibration {

using state::Micros;

constexpr std::size_t kMaxMeasurements = 8;
constexpr std::size_t kMaxRevolutions = 50;

enum class AxisScaleSource : std::uint8_t {
    ConfigNominal,
    SavedNominal,
    User,
};

enum class Persistence : std::uint8_t {
    Absent,
    Verified,
    Failed,
    Unconfirmed,
};

enum class Repeatability : std::uint8_t {
    Insufficient,
    Good,
    Warn,
    Invalid,
};

enum class SessionState : std::uint8_t {
    Inactive,
    Preparing,
    InitialPositioning,
    AwaitingStartStop,
    StartReady,
    Measuring,
    AwaitingEndStop,
    EndReady,
    Review,
    Applying,
    Saving,
    Saved,
    SaveFailed,
    SaveUncertain,
    Cancelling,
    Cancelled,
    Error,
};

enum class InvalidReason : std::uint8_t {
    Ok,
    ZeroDelta,
    DirectionMismatch,
    PositionUntrusted,
    Fault,
    Reversal,
    EmergencyStop,
    StopUnconfirmed,
    EpochChanged,
    LeaseExpired,
    HoldLost,
    HoldUnavailable,
    ClientAbort,
    InvalidResult,
};

enum class SessionError : std::uint8_t {
    None,
    ControlBusy,
    HoldUnavailable,
    DirectionViolation,
    ClientLeaseExpired,
    StopUnconfirmed,
    ReferenceConfirmTimeout,
    SaveFailed,
    SaveUncertain,
    MeasurementInvalid,
    SessionExpired,
};

const char *sessionStateName(SessionState state) noexcept;
const char *sessionErrorName(SessionError error) noexcept;
const char *invalidReasonName(InvalidReason reason) noexcept;
const char *repeatabilityName(Repeatability value) noexcept;
const char *persistenceName(Persistence value) noexcept;
const char *sourceName(AxisScaleSource value) noexcept;

/// Постоянный коэффициент оси. Угол хранится в наноградусах на полушаг.
struct AxisScale {
    std::int64_t nanodegrees_per_half_step = 0;
    AxisScaleSource source = AxisScaleSource::ConfigNominal;
    Persistence persistence = Persistence::Absent;
    bool saved = false;
    std::uint32_t sample_count = 0;
    std::uint32_t total_revolutions = 0;
    std::uint32_t spread_ppm = 0;

    bool userCalibrated() const noexcept
    {
        return source == AxisScaleSource::User;
    }
};

struct Measurement {
    std::uint32_t index = 0;
    std::uint32_t revolutions = 0;
    std::int64_t start_position = 0;
    std::int64_t end_position = 0;
    std::uint64_t delta_half_steps = 0;
    double half_steps_per_revolution = 0.0;
    double degrees_per_half_step = 0.0;
    std::int32_t deviation_ppm = 0;
    bool valid = false;
    InvalidReason reason = InvalidReason::InvalidResult;
    std::uint64_t start_ticket = 0;
    std::uint64_t stop_ticket = 0;
    std::uint32_t epoch = 0;
};

struct SessionConfig {
    motion::AxisId axis = motion::AxisId::Azimuth;
    motion::Direction direction = motion::Direction::Forward;
    std::uint32_t revolutions = 1;
    std::uint32_t repeats = 2;
};

struct CalibrationParams {
    std::array<std::uint32_t, 2> nominal_half_steps_per_revolution = {4096, 4096};
    std::uint32_t min_revolutions = 1;
    std::uint32_t max_revolutions = 20;
    std::uint32_t min_measurements = 2;
    std::uint32_t max_measurements = kMaxMeasurements;
    std::uint32_t max_spread_ppm = 5000;
    double min_degrees_per_half_step = 1.0e-5;
    double max_degrees_per_half_step = 1.0;
};

bool validParams(const CalibrationParams &params) noexcept;
bool validConfig(const SessionConfig &config, const CalibrationParams &params) noexcept;

}  // namespace calibration
