/**
 * @file shutdown_protocol.cpp
 * @brief Переходы протокола завершения задачи-супервизора.
 */

#include "web/shutdown_protocol.hpp"

namespace web {

ShutdownProtocol::WaitAction ShutdownProtocol::beginStop() noexcept
{
    switch (state_) {
        case State::Idle:
            state_ = State::Requested;
            ++requests_;
            return WaitAction::NotifyThenWait;
        case State::Requested:
            // Таймаут уже был или ожидание продолжается: уведомлять нельзя.
            return WaitAction::WaitOnly;
        case State::Completed:
            return WaitAction::Completed;
    }
    return WaitAction::Completed;
}

void ShutdownProtocol::onWaitTimeout() noexcept
{
    if (state_ != State::Requested) {
        return;
    }
    ++timeouts_;
    // Состояние остаётся Requested: следующий вызов только ждёт.
}

void ShutdownProtocol::onCompleted() noexcept
{
    if (state_ != State::Requested) {
        return;
    }
    state_ = State::Completed;
    ++completions_;
}

void ShutdownProtocol::reset() noexcept
{
    state_ = State::Idle;
}

}  // namespace web
