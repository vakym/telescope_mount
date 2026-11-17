/**
 * @file motor_service.hpp
 * @brief Точка входа слоя движения для main, input и comms.
 */

#pragma once

#include <array>
#include <cstdint>

#include "board/board.hpp"
#include "esp_err.h"
#include "motion/axis_config.hpp"
#include "motion/motion_controller.hpp"
#include "motor_hal/axis_motor.hpp"

namespace motor_hal {

/// Результаты пары осей для одной операции.
struct PairReceipt {
    CommandReceipt azimuth;
    CommandReceipt altitude;
};

class MotorService {
public:
    esp_err_t start(const board::MotorPins &pins, const motion::AxisConfig (&configs)[2]);
    esp_err_t stop();

    /// Команды, подтверждаемые тикетом: возвращают идентификатор и эпоху.
    CommandReceipt enable(motion::AxisId id);
    CommandReceipt setTargetRate(motion::AxisId id, float half_steps_per_s);
    CommandReceipt stopAxis(motion::AxisId id, motion::StopMode mode);
    CommandReceipt resetFault(motion::AxisId id);
    PairReceipt stopAll(motion::StopMode mode);

    /// Команды без подтверждения тикетом на этом слое.
    bool disable(motion::AxisId id);
    bool moveSteps(motion::AxisId id, std::int64_t half_steps);
    bool release(motion::AxisId id);
    bool energize(motion::AxisId id);
    bool setHoldPolicy(motion::AxisId id, const motion::HoldPolicy &policy);
    /// То же, но с тикетом: позволяет подтвердить фактическое применение.
    CommandReceipt setHoldPolicyCommand(motion::AxisId id, const motion::HoldPolicy &policy);

    /// Последняя принятая политика удержания. Читает и пишет управляющая
    /// задача; политика нужна для сохранения исходного значения при
    /// временном удержании опорной точки.
    motion::HoldPolicy holdPolicy(motion::AxisId id) const;

    motion::AxisStatus status(motion::AxisId id) const;

    /// Диагностика канала приоритетного останова по обеим осям.
    std::uint32_t emergencySlotBusy() const noexcept;
    std::uint32_t notifyFailures() const noexcept;

private:
    /// Idle — можно запускать; Running — работает; Stopping — не все оси
    /// подтвердили останов, повторный stop() обязателен до нового start().
    enum class State : std::uint8_t { Idle, Running, Stopping };

    static constexpr std::size_t kAxisCount = static_cast<std::size_t>(motion::AxisId::Count);
    static bool valid(motion::AxisId id) noexcept;

    motion::MotionController motion_{};
    std::array<AxisMotor, kAxisCount> motors_{};
    std::array<motion::HoldPolicy, kAxisCount> hold_policy_{};
    State state_ = State::Idle;
};

}  // namespace motor_hal
