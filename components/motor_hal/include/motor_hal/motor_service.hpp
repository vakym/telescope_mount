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

class MotorService {
public:
    esp_err_t start(const board::MotorPins &pins, const motion::AxisConfig (&configs)[2]);
    esp_err_t stop();

    bool enable(motion::AxisId id);
    bool disable(motion::AxisId id);
    bool setTargetRate(motion::AxisId id, float half_steps_per_s);
    bool moveSteps(motion::AxisId id, std::int64_t half_steps);
    bool stopAxis(motion::AxisId id, motion::StopMode mode);
    bool stopAll(motion::StopMode mode);
    bool release(motion::AxisId id);
    bool energize(motion::AxisId id);
    bool setHoldPolicy(motion::AxisId id, const motion::HoldPolicy &policy);
    bool resetFault(motion::AxisId id);

    motion::AxisStatus status(motion::AxisId id) const;

private:
    /// Idle — можно запускать; Running — работает; Stopping — не все оси
    /// подтвердили останов, повторный stop() обязателен до нового start().
    enum class State : std::uint8_t { Idle, Running, Stopping };

    static constexpr std::size_t kAxisCount = static_cast<std::size_t>(motion::AxisId::Count);
    static bool valid(motion::AxisId id) noexcept;

    motion::MotionController motion_{};
    std::array<AxisMotor, kAxisCount> motors_{};
    State state_ = State::Idle;
};

}  // namespace motor_hal
