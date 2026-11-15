/**
 * @file ap_lifecycle.cpp
 * @brief Переходы координатора: операции, события, поздние подтверждения.
 */

#include "wifi/ap_lifecycle.hpp"

namespace wifi {
namespace {

constexpr std::uint8_t kStationsMax = 0xFFU;

}  // namespace

void ApCoordinator::beginStartup() noexcept
{
    lifecycle_ = Lifecycle::Starting;
    radio_ = RadioState::Unknown;
    operation_ = Operation::None;
}

void ApCoordinator::startupSucceeded(bool ap_confirmed_running) noexcept
{
    lifecycle_ = Lifecycle::Running;
    radio_ = ap_confirmed_running ? RadioState::Running : RadioState::Stopped;
    operation_ = Operation::None;
}

void ApCoordinator::startupFailed() noexcept
{
    lifecycle_ = Lifecycle::Stopped;
    radio_ = RadioState::Unknown;
    operation_ = Operation::None;
}

Action ApCoordinator::beginInitialStart() noexcept
{
    if (lifecycle_ != Lifecycle::Starting || operation_ != Operation::None) {
        return Action::None;
    }
    if (radio_ != RadioState::Unknown) {
        return Action::None;
    }
    operation_ = Operation::Starting;
    return Action::StartRadio;
}

Action ApCoordinator::beginToggleOperation() noexcept
{
    if (lifecycle_ != Lifecycle::Running || operation_ != Operation::None) {
        return Action::None;
    }
    switch (radio_) {
        case RadioState::Stopped:
            operation_ = Operation::Starting;
            return Action::StartRadio;
        case RadioState::Running:
            operation_ = Operation::Stopping;
            return Action::StopRadio;
        case RadioState::Unknown:
            break;
    }
    return Action::None;
}

Action ApCoordinator::beginResync() noexcept
{
    if (operation_ != Operation::None) {
        return Action::None;
    }
    operation_ = Operation::Resyncing;
    ++resyncs;
    return Action::ResyncStop;
}

void ApCoordinator::onApStarted() noexcept
{
    const bool was_running = radio_ == RadioState::Running;
    radio_ = RadioState::Running;
    // Событие завершает только операцию запуска; позднее или дублирующее
    // событие не подтверждает противоположную операцию.
    if (operation_ == Operation::Starting) {
        operation_ = Operation::None;
    }
    if (!was_running) {
        ++starts;
    }
}

void ApCoordinator::onApStopped() noexcept
{
    const bool was_stopped = radio_ == RadioState::Stopped;
    radio_ = RadioState::Stopped;
    stations_ = 0;
    if (operation_ == Operation::Stopping ||
        operation_ == Operation::Resyncing) {
        operation_ = Operation::None;
    }
    if (!was_stopped) {
        ++stops;
    }
}

void ApCoordinator::onOperationTimeout() noexcept
{
    operation_ = Operation::None;
    radio_ = RadioState::Unknown;
    ++event_timeouts;
}

void ApCoordinator::onOperationFailed() noexcept
{
    operation_ = Operation::None;
    radio_ = RadioState::Unknown;
    ++operation_errors;
}

void ApCoordinator::onAlreadyStopped() noexcept
{
    radio_ = RadioState::Stopped;
    operation_ = Operation::None;
}

void ApCoordinator::onStaConnected() noexcept
{
    if (stations_ < kStationsMax) {
        ++stations_;
    }
}

void ApCoordinator::onStaDisconnected() noexcept
{
    if (stations_ > 0) {
        --stations_;
    }
}

bool ApCoordinator::beginShutdown() noexcept
{
    if (lifecycle_ == Lifecycle::Stopping) {
        return false;
    }
    lifecycle_ = Lifecycle::Stopping;
    operation_ = Operation::None;
    return true;
}

void ApCoordinator::finishShutdown() noexcept
{
    lifecycle_ = Lifecycle::Stopped;
    radio_ = RadioState::Unknown;
    operation_ = Operation::None;
}

}  // namespace wifi
