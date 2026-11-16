/**
 * @file server_lifecycle.cpp
 * @brief Переходы автомата запуска и останова HTTP-сервера.
 */

#include "web/server_lifecycle.hpp"

namespace web {

ServerAction ServerLifecycle::update(bool ap_running, bool ap_stopping,
                                     bool retry_elapsed) noexcept
{
    switch (state_) {
        case ServerState::Stopped:
            if (ap_running && !ap_stopping) {
                state_ = ServerState::Starting;
                ++starts;
                return ServerAction::Start;
            }
            return ServerAction::None;

        case ServerState::Starting:
            // Результат запуска приходит синхронно в том же тике.
            return ServerAction::None;

        case ServerState::Running:
            if (!ap_running || ap_stopping) {
                state_ = ServerState::Stopping;
                stop_retry_pending_ = false;
                ++stops;
                return ServerAction::Stop;
            }
            return ServerAction::None;

        case ServerState::Stopping:
            if (stop_retry_pending_ && retry_elapsed) {
                stop_retry_pending_ = false;
                ++stops;
                return ServerAction::Stop;
            }
            return ServerAction::None;

        case ServerState::StartFailed:
            if (ap_running && !ap_stopping && retry_elapsed) {
                state_ = ServerState::Starting;
                ++starts;
                return ServerAction::Start;
            }
            return ServerAction::None;

        case ServerState::StopFailed:
            // Сервер мог остаться живым: второй экземпляр запрещён.
            return ServerAction::None;
    }
    return ServerAction::None;
}

void ServerLifecycle::startSucceeded() noexcept
{
    if (state_ != ServerState::Starting) {
        return;
    }
    state_ = ServerState::Running;
    ++starts_confirmed;
}

void ServerLifecycle::startFailed() noexcept
{
    if (state_ != ServerState::Starting) {
        return;
    }
    state_ = ServerState::StartFailed;
    ++start_failures;
}

void ServerLifecycle::startFailedWithLiveServer(bool retry_allowed) noexcept
{
    if (state_ != ServerState::Starting) {
        return;
    }
    ++start_failures;
    ++stops;
    ++stop_unconfirmed;
    if (retry_allowed) {
        // Останов уже отправлялся; остаётся дождаться подтверждения.
        state_ = ServerState::Stopping;
        stop_retry_pending_ = true;
        return;
    }
    state_ = ServerState::StopFailed;
    stop_retry_pending_ = false;
    ++stop_terminal;
}

void ServerLifecycle::stopSucceeded() noexcept
{
    if (state_ != ServerState::Stopping && state_ != ServerState::Running) {
        return;
    }
    state_ = ServerState::Stopped;
    stop_retry_pending_ = false;
    ++stops_confirmed;
}

void ServerLifecycle::stopUnconfirmed(bool retry_allowed) noexcept
{
    if (state_ != ServerState::Stopping) {
        return;
    }
    ++stop_unconfirmed;
    if (retry_allowed) {
        // Сервер по контракту жив: повтор останова разрешён и ожидается.
        stop_retry_pending_ = true;
        return;
    }
    state_ = ServerState::StopFailed;
    stop_retry_pending_ = false;
    ++stop_terminal;
}

}  // namespace web
