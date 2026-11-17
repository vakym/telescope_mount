/**
 * @file actuator.hpp
 * @brief Порт исполнения: единственная точка отправки команд в моторы.
 *
 * Реализация для ESP-IDF (control_hal::MotorAxisPort) оборачивает
 * motor_hal::MotorService. Тестовая реализация — в test/host. Чистая логика
 * не знает о MotorService и FreeRTOS.
 */

#pragma once

#include "control/command_ticket.hpp"
#include "motion/axis_config.hpp"
#include "motion/axis_planner.hpp"
#include "motion/types.hpp"

namespace control {

class AxisActuator {
public:
    virtual ~AxisActuator() = default;

    virtual Delivery enable(motion::AxisId id) = 0;
    virtual Delivery setRate(motion::AxisId id, float half_steps_per_s) = 0;
    virtual Delivery stopSmooth(motion::AxisId id) = 0;
    virtual Delivery stopImmediate(motion::AxisId id) = 0;
    virtual Delivery resetFault(motion::AxisId id) = 0;

    /// Временная политика удержания опорной точки (калибровка).
    /// По умолчанию не поддерживается: тестовые порты не обязаны её
    /// реализовывать, а вызывающий обязан проверить false.
    virtual bool setHoldPolicy(motion::AxisId id, const motion::HoldPolicy &policy)
    {
        (void)id;
        (void)policy;
        return false;
    }

    /// Текущая политика удержания оси (для сохранения и восстановления).
    virtual motion::HoldPolicy holdPolicy(motion::AxisId id) const
    {
        (void)id;
        return motion::HoldPolicy{};
    }

    virtual motion::AxisStatus status(motion::AxisId id) const = 0;
};

}  // namespace control
