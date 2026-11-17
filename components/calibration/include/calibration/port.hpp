/**
 * @file port.hpp
 * @brief Чистый интерфейс управления, который нужен сервису калибровки.
 *
 * Реализация живёт в calibration_hal и преобразует вызовы в API
 * ControlService. Прямого доступа к MotorService у калибровки нет.
 */

#pragma once

#include <cstdint>

#include "calibration/types.hpp"
#include "motion/types.hpp"
#include "state/channels.hpp"

namespace calibration {

/// Доказательство подтверждённого останова, полученное от управляющего слоя.
struct StopAttestation {
    bool valid = false;
    std::uint64_t request_id = 0;
    std::uint64_t ticket_id[2] = {0, 0};
    std::uint32_t epoch[2] = {0, 0};
    std::int64_t position_step[2] = {0, 0};
    Micros requested_us = 0;
    Micros confirmed_us = 0;
    Micros polled_us[2] = {0, 0};
    bool position_trusted[2] = {false, false};
    bool energized[2] = {false, false};
    bool immediate = false;
    bool confirmed = false;
    std::uint32_t motion_generation = 0;
};

/// Минимальный снимок контура для сервиса калибровки.
struct Snapshot {
    std::int64_t position_step[2] = {0, 0};
    bool position_trusted[2] = {true, true};
    bool has_fault[2] = {false, false};
    bool energized[2] = {false, false};
    motion::AxisState state[2] = {motion::AxisState::Disabled,
                                  motion::AxisState::Disabled};
    std::uint32_t command_epoch[2] = {0, 0};
    float rate[2] = {0.0F, 0.0F};
    state::SourceId owner = state::SourceId::None;
    state::ControlPhase phase = state::ControlPhase::Disarmed;
    bool permit = false;
    std::uint32_t owner_generation = 0;
    std::uint32_t motion_generation = 0;
    bool input_ready = false;
    bool flash_window_active = false;
};

class CalibrationPort {
public:
    virtual ~CalibrationPort() = default;

    virtual bool claim(std::uint32_t session) = 0;
    virtual void release(std::uint32_t session) = 0;
    virtual void setInputReady(bool ready, std::uint32_t session,
                               Micros valid_until_us) = 0;
    virtual bool beginHold(motion::AxisId axis, std::uint32_t session,
                           Micros absolute_deadline_us) = 0;
    virtual bool endHold(motion::AxisId axis, std::uint32_t session) = 0;
    /// Применено ли временное удержание этой сессии (подтверждение снятия).
    virtual bool holdActive(motion::AxisId axis, std::uint32_t session) const = 0;
    virtual bool setRate(std::uint32_t session, motion::AxisId axis, float rate,
                         Micros lease_us) = 0;
    virtual bool requestStop(std::uint64_t request_id) = 0;
    virtual StopAttestation lastStopAttestation() const = 0;
    virtual bool beginFlashWindow(Micros timeout_us) = 0;
    virtual void endFlashWindow() = 0;
    virtual Snapshot snapshot() const = 0;
    virtual Micros nowUs() const = 0;
};

}  // namespace calibration
