/**
 * @file retry_gate.cpp
 * @brief Реализация отсчёта интервала повторов.
 */

#include "web/retry_gate.hpp"

namespace web {

void RetryGate::noteAttempt(std::uint32_t now_ticks) noexcept
{
    armed_at_ = now_ticks;
    armed_ = true;
    ++attempts_;
}

void RetryGate::clear() noexcept
{
    armed_ = false;
    armed_at_ = 0;
}

bool RetryGate::elapsed(std::uint32_t now_ticks,
                        std::uint32_t interval_ticks) const noexcept
{
    if (!armed_) {
        return true;
    }
    // Беззнаковая разность корректна и при переполнении счётчика тиков.
    return (now_ticks - armed_at_) >= interval_ticks;
}

}  // namespace web
