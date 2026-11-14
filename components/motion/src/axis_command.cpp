/**
 * @file axis_command.cpp
 * @brief Фабрики команд оси.
 */

#include "motion/axis_command.hpp"

namespace motion {

AxisCommand AxisCommand::enable() noexcept
{
    AxisCommand cmd;
    cmd.kind = CommandKind::Enable;
    return cmd;
}

AxisCommand AxisCommand::disable() noexcept
{
    AxisCommand cmd;
    cmd.kind = CommandKind::Disable;
    return cmd;
}

AxisCommand AxisCommand::setTargetRate(float half_steps_per_s) noexcept
{
    AxisCommand cmd;
    cmd.kind = CommandKind::SetTargetRate;
    cmd.value = half_steps_per_s;
    return cmd;
}

AxisCommand AxisCommand::moveSteps(std::int64_t half_steps) noexcept
{
    AxisCommand cmd;
    cmd.kind = CommandKind::MoveSteps;
    cmd.steps = half_steps;
    return cmd;
}

AxisCommand AxisCommand::stop(StopMode mode) noexcept
{
    AxisCommand cmd;
    cmd.kind = mode == StopMode::Smooth ? CommandKind::StopSmooth : CommandKind::StopImmediate;
    return cmd;
}

AxisCommand AxisCommand::release() noexcept
{
    AxisCommand cmd;
    cmd.kind = CommandKind::Release;
    return cmd;
}

AxisCommand AxisCommand::energize() noexcept
{
    AxisCommand cmd;
    cmd.kind = CommandKind::Energize;
    return cmd;
}

AxisCommand AxisCommand::setHoldPolicy(const HoldPolicy &policy) noexcept
{
    AxisCommand cmd;
    cmd.kind = CommandKind::SetHoldPolicy;
    cmd.hold = policy;
    return cmd;
}

AxisCommand AxisCommand::resetFault() noexcept
{
    AxisCommand cmd;
    cmd.kind = CommandKind::ResetFault;
    return cmd;
}

}  // namespace motion
