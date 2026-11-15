/**
 * @file motor_axis_port.hpp
 * @brief Тонкий адаптер: единственная точка команд в MotorService.
 *
 * Идентификаторы назначает нижний слой (AxisMotor), поэтому адаптер не имеет
 * собственного генератора и не может его сбросить при перезапуске службы.
 */

#pragma once

#include "control/actuator.hpp"
#include "motor_hal/motor_service.hpp"

namespace control_hal {

class MotorAxisPort final : public control::AxisActuator {
public:
    explicit MotorAxisPort(motor_hal::MotorService &motors) noexcept;

    control::Delivery enable(motion::AxisId id) override;
    control::Delivery setRate(motion::AxisId id, float half_steps_per_s) override;
    control::Delivery stopSmooth(motion::AxisId id) override;
    control::Delivery stopImmediate(motion::AxisId id) override;
    control::Delivery resetFault(motion::AxisId id) override;

    motion::AxisStatus status(motion::AxisId id) const override;

private:
    static control::Delivery convert(const motor_hal::CommandReceipt &receipt) noexcept;

    motor_hal::MotorService &motors_;
};

}  // namespace control_hal
