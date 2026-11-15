/**
 * @file input_grace.cpp
 * @brief Реализация ограниченного удержания при единичной ошибке ADC.
 */

#include "control/input_grace.hpp"

#include <algorithm>

namespace control {

bool InputGracePolicy::configure(const Config &config,
                                 state::Micros stale_after_us) noexcept
{
    if (config.grace_us == 0U || stale_after_us == 0U) {
        return false;
    }
    config_ = config;
    stale_after_us_ = stale_after_us;
    reset();
    return true;
}

void InputGracePolicy::reset() noexcept
{
    active_ = false;
    started_us_ = 0;
    last_good_us_ = 0;
    have_good_ = false;
}

void InputGracePolicy::onGoodSample(state::Micros measured_us) noexcept
{
    active_ = false;
    started_us_ = 0;
    have_good_ = true;
    last_good_us_ = measured_us;
}

state::Micros InputGracePolicy::deadline() const noexcept
{
    if (!active_ || !have_good_) {
        return 0;
    }
    const state::Micros by_grace = started_us_ + config_.grace_us;
    const state::Micros by_age = last_good_us_ + stale_after_us_;
    return std::min(by_grace, by_age);
}

InputGracePolicy::Decision InputGracePolicy::onRejected(state::DenyReason reason,
                                                        state::Micros now_us) noexcept
{
    Decision decision{};
    // Инициировать движение grace не может: без последнего хорошего
    // измерения удерживать нечего.
    if (reason != state::DenyReason::InvalidSample || !have_good_) {
        active_ = false;
        decision.expired = true;
        return decision;
    }
    if (!active_) {
        active_ = true;
        started_us_ = now_us;
    }
    const state::Micros limit = deadline();
    if (limit == 0U || now_us >= limit) {
        active_ = false;
        decision.expired = true;
        return decision;
    }
    decision.hold_previous = true;
    return decision;
}

}  // namespace control
