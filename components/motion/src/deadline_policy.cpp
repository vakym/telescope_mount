/**
 * @file deadline_policy.cpp
 * @brief Реализация чистых функций расписания шагов.
 */

#include "motion/deadline_policy.hpp"

#include <algorithm>
#include <limits>

namespace motion {
namespace {

/// Насыщающее сложение: дедлайны не должны заворачиваться.
constexpr std::uint64_t satAdd(std::uint64_t a, std::uint64_t b) noexcept
{
    const std::uint64_t max = std::numeric_limits<std::uint64_t>::max();
    return (a > max - b) ? max : a + b;
}

}  // namespace

StepDue classifyWakeup(std::uint64_t now_us,
                       std::uint64_t pending_deadline_us,
                       bool has_pending,
                       std::uint64_t max_lateness_us) noexcept
{
    if (!has_pending) {
        return StepDue::IgnoreStale;
    }
    if (now_us < pending_deadline_us) {
        return StepDue::IgnoreSpurious;
    }
    if (now_us - pending_deadline_us > max_lateness_us) {
        return StepDue::MissedTooFar;
    }
    return StepDue::ExecuteNow;
}

DeadlineDecision planNextStep(std::uint64_t now_us,
                              std::uint64_t next_deadline_us,
                              std::uint64_t last_switch_us,
                              std::uint32_t interval_us,
                              std::uint32_t min_phase_interval_us,
                              std::uint32_t min_alarm_lead_us,
                              std::uint32_t late_tolerance_us,
                              DeadlineMissPolicy policy) noexcept
{
    DeadlineDecision decision;

    const std::uint64_t earliest_phase = satAdd(last_switch_us, min_phase_interval_us);
    const std::uint64_t earliest_arm = satAdd(now_us, min_alarm_lead_us);
    const std::uint64_t desired = std::max({next_deadline_us, earliest_phase, earliest_arm});

    decision.deadline_us = desired;

    if (desired == next_deadline_us) {
        decision.action = DeadlineAction::ArmAtDeadline;
        return decision;
    }

    const std::uint64_t shift = desired - next_deadline_us;
    if (shift <= late_tolerance_us) {
        decision.action = DeadlineAction::ArmAtDeadline;
        decision.count_correction = true;
        return decision;
    }

    if (policy == DeadlineMissPolicy::Fault) {
        decision.action = DeadlineAction::Fault;
        return decision;
    }

    // Полноценный Replan: следующий шаг не раньше штатного интервала от now.
    const std::uint64_t replan = satAdd(now_us, interval_us);
    decision.action = DeadlineAction::ReplanFromNow;
    decision.deadline_us = std::max({replan, earliest_phase, earliest_arm});
    decision.count_replan = true;
    return decision;
}

}  // namespace motion
