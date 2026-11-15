/**
 * @file toggle_admission.cpp
 * @brief Атомарное резервирование заявок и раздельный учёт их исходов.
 */

#include "wifi/toggle_admission.hpp"

namespace wifi {

void ToggleAdmission::increment(std::atomic<std::uint32_t> &counter) noexcept
{
    counter.fetch_add(1, std::memory_order_relaxed);
}

ToggleAdmission::Outcome ToggleAdmission::tryAdmit(Lifecycle lifecycle,
                                                   RadioState radio) noexcept
{
    Outcome outcome;

    if (lifecycle != Lifecycle::Running) {
        increment(rejected_not_running);
        outcome.result = Result::NotRunning;
        return outcome;
    }

    if (radio == RadioState::Unknown) {
        increment(rejected_unconfirmed);
        outcome.result = Result::Unconfirmed;
        // Отказ в переключении не должен оставить запрос ресинхронизации
        // без пробуждения задачи.
        outcome.wake_task = true;
        return outcome;
    }

    std::uint8_t current = pending_.load(std::memory_order_relaxed);
    for (;;) {
        if (current >= kMaxPendingToggles) {
            increment(rejected_full);
            outcome.result = Result::Full;
            return outcome;
        }
        const auto next = static_cast<std::uint8_t>(current + 1);
        if (pending_.compare_exchange_weak(current, next,
                                           std::memory_order_acq_rel,
                                           std::memory_order_relaxed)) {
            increment(accepted);
            outcome.result = Result::Accepted;
            outcome.wake_task = true;
            return outcome;
        }
    }
}

bool ToggleAdmission::takeOne() noexcept
{
    std::uint8_t current = pending_.load(std::memory_order_relaxed);
    while (current > 0) {
        const auto next = static_cast<std::uint8_t>(current - 1);
        if (pending_.compare_exchange_weak(current, next,
                                           std::memory_order_acq_rel,
                                           std::memory_order_relaxed)) {
            return true;
        }
    }
    return false;
}

std::uint8_t ToggleAdmission::dropPending(DropReason reason) noexcept
{
    const std::uint8_t count = pending_.exchange(0, std::memory_order_acq_rel);
    if (count == 0) {
        return 0;
    }
    switch (reason) {
        case DropReason::Shutdown:
            dropped_on_shutdown.fetch_add(count, std::memory_order_relaxed);
            break;
        case DropReason::Timeout:
            dropped_on_timeout.fetch_add(count, std::memory_order_relaxed);
            break;
        case DropReason::Error:
            dropped_on_error.fetch_add(count, std::memory_order_relaxed);
            break;
    }
    return count;
}

void ToggleAdmission::accountExecuted() noexcept
{
    increment(executed);
}

void ToggleAdmission::accountFailed(DropReason reason) noexcept
{
    switch (reason) {
        case DropReason::Shutdown:
            increment(failed_on_shutdown);
            break;
        case DropReason::Timeout:
            increment(failed_on_timeout);
            break;
        case DropReason::Error:
            increment(failed_on_error);
            break;
    }
}

std::uint8_t ToggleAdmission::pending() const noexcept
{
    return pending_.load(std::memory_order_relaxed);
}

}  // namespace wifi
