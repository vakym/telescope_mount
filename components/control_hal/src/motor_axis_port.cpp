/**
 * @file motor_axis_port.cpp
 * @brief Реализация адаптера исполнения.
 */

#include "control_hal/motor_axis_port.hpp"

namespace control_hal {

MotorAxisPort::MotorAxisPort(motor_hal::MotorService &motors) noexcept : motors_(motors)
{
}

control::Delivery MotorAxisPort::convert(const motor_hal::CommandReceipt &receipt) noexcept
{
    control::Delivery delivery{};
    delivery.accepted = receipt.accepted;
    delivery.notified = receipt.notified;
    delivery.ticket.id = receipt.command_id;
    delivery.ticket.epoch = receipt.epoch;
    return delivery;
}

control::Delivery MotorAxisPort::enable(motion::AxisId id)
{
    return convert(motors_.enable(id));
}

control::Delivery MotorAxisPort::setRate(motion::AxisId id, float half_steps_per_s)
{
    return convert(motors_.setTargetRate(id, half_steps_per_s));
}

control::Delivery MotorAxisPort::stopSmooth(motion::AxisId id)
{
    return convert(motors_.stopAxis(id, motion::StopMode::Smooth));
}

control::Delivery MotorAxisPort::stopImmediate(motion::AxisId id)
{
    return convert(motors_.stopAxis(id, motion::StopMode::Immediate));
}

control::Delivery MotorAxisPort::resetFault(motion::AxisId id)
{
    return convert(motors_.resetFault(id));
}

bool MotorAxisPort::setHoldPolicy(motion::AxisId id, const motion::HoldPolicy &policy)
{
    return motors_.setHoldPolicy(id, policy);
}

motion::HoldPolicy MotorAxisPort::holdPolicy(motion::AxisId id) const
{
    return motors_.holdPolicy(id);
}

motion::AxisStatus MotorAxisPort::status(motion::AxisId id) const
{
    return motors_.status(id);
}

}  // namespace control_hal
