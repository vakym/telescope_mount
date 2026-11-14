/**
 * @file motion_controller.hpp
 * @brief Решения по двум осям: независимые планировщики без блокировок.
 *
 * Класс не знает ни о GPIO, ни о времени. Единственный писатель каждого
 * планировщика — задача соответствующей оси; снимок статуса для читателей
 * формирует исполнитель.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "motion/axis_planner.hpp"
#include "motion/types.hpp"

namespace motion {

class MotionController {
public:
    bool configure(AxisId id, const AxisConfig &config) noexcept;

    bool apply(AxisId id, const AxisCommand &cmd) noexcept;

    bool hasPending(AxisId id) const noexcept;
    bool wantsStep(AxisId id) const noexcept;
    bool prepareNextStep(AxisId id, StepPlan &out) noexcept;
    void commitPending(AxisId id) noexcept;
    void cancelPendingStep(AxisId id) noexcept;

    void forceReleased(AxisId id, ReleaseReason reason) noexcept;
    void enterFault(AxisId id, FaultReason reason) noexcept;
    void resetFault(AxisId id) noexcept;

    std::uint32_t planKey(AxisId id) const noexcept;
    AxisStatus status(AxisId id) const noexcept;

    AxisPlanner &planner(AxisId id) noexcept;
    const AxisPlanner &planner(AxisId id) const noexcept;

private:
    static constexpr std::size_t kAxisCount = static_cast<std::size_t>(AxisId::Count);
    static bool valid(AxisId id) noexcept;

    std::array<AxisPlanner, kAxisCount> planners_{};
};

}  // namespace motion
