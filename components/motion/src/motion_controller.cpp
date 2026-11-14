/**
 * @file motion_controller.cpp
 * @brief Делегирование операций планировщикам осей.
 */

#include "motion/motion_controller.hpp"

namespace motion {

bool MotionController::valid(AxisId id) noexcept
{
    return static_cast<std::uint8_t>(id) < static_cast<std::uint8_t>(AxisId::Count);
}

bool MotionController::configure(AxisId id, const AxisConfig &config) noexcept
{
    return valid(id) && planners_[static_cast<std::size_t>(id)].configure(config);
}

bool MotionController::apply(AxisId id, const AxisCommand &cmd) noexcept
{
    return valid(id) && planners_[static_cast<std::size_t>(id)].apply(cmd);
}

bool MotionController::hasPending(AxisId id) const noexcept
{
    return valid(id) && planners_[static_cast<std::size_t>(id)].hasPending();
}

bool MotionController::wantsStep(AxisId id) const noexcept
{
    return valid(id) && planners_[static_cast<std::size_t>(id)].wantsStep();
}

bool MotionController::prepareNextStep(AxisId id, StepPlan &out) noexcept
{
    return valid(id) && planners_[static_cast<std::size_t>(id)].prepareNextStep(out);
}

void MotionController::commitPending(AxisId id) noexcept
{
    if (valid(id)) {
        planners_[static_cast<std::size_t>(id)].commitPending();
    }
}

void MotionController::cancelPendingStep(AxisId id) noexcept
{
    if (valid(id)) {
        planners_[static_cast<std::size_t>(id)].cancelPendingStep();
    }
}

void MotionController::forceReleased(AxisId id, ReleaseReason reason) noexcept
{
    if (valid(id)) {
        planners_[static_cast<std::size_t>(id)].forceReleased(reason);
    }
}

void MotionController::enterFault(AxisId id, FaultReason reason) noexcept
{
    if (valid(id)) {
        planners_[static_cast<std::size_t>(id)].enterFault(reason);
    }
}

void MotionController::resetFault(AxisId id) noexcept
{
    if (valid(id)) {
        planners_[static_cast<std::size_t>(id)].resetFault();
    }
}

std::uint32_t MotionController::planKey(AxisId id) const noexcept
{
    return valid(id) ? planners_[static_cast<std::size_t>(id)].planKey() : 0U;
}

AxisStatus MotionController::status(AxisId id) const noexcept
{
    if (!valid(id)) {
        return AxisStatus{};
    }
    return planners_[static_cast<std::size_t>(id)].status();
}

AxisPlanner &MotionController::planner(AxisId id) noexcept
{
    return planners_[static_cast<std::size_t>(id)];
}

const AxisPlanner &MotionController::planner(AxisId id) const noexcept
{
    return planners_[static_cast<std::size_t>(id)];
}

}  // namespace motion
