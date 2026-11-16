/**
 * @file server_controller.cpp
 * @brief Применение правил автомата к результатам реального сервера.
 */

#include "web/server_controller.hpp"

namespace web {

ServerAction ServerController::tick(bool ap_running, bool ap_stopping,
                                    bool retry_elapsed) noexcept
{
    const ServerAction action =
        lifecycle_.update(ap_running, ap_stopping, retry_elapsed);

    switch (action) {
        case ServerAction::Start: {
            retry_pending_ = false;
            switch (port_.start()) {
                case StartOutcome::Started:
                    lifecycle_.startSucceeded();
                    break;
                case StartOutcome::Failed:
                    lifecycle_.startFailed();
                    retry_pending_ = true;
                    break;
                case StartOutcome::FailedServerLiveRetryable:
                    lifecycle_.startFailedWithLiveServer(true);
                    retry_pending_ = true;
                    break;
                case StartOutcome::FailedServerLiveTerminal:
                    lifecycle_.startFailedWithLiveServer(false);
                    retry_pending_ = false;
                    break;
            }
            break;
        }
        case ServerAction::Stop:
            retry_pending_ = false;
            applyStop(port_.stop());
            break;
        case ServerAction::None:
            break;
    }
    return action;
}

ShutdownOutcome ServerController::shutdownOnce() noexcept
{
    // Терминальный останов не повторяется вслепую: код возврата библиотеки
    // не позволяет утверждать, что повтор безопасен.
    if (lifecycle_.state() == ServerState::StopFailed) {
        return ShutdownOutcome::Terminal;
    }

    if (!port_.hasServer()) {
        // Сервера нет: подтверждать нечего, состояние приводится к Stopped.
        if (lifecycle_.state() == ServerState::Running ||
            lifecycle_.state() == ServerState::Stopping) {
            lifecycle_.stopSucceeded();
        }
        retry_pending_ = false;
        return ShutdownOutcome::Clean;
    }

    // Финализация идёт теми же переходами, что и обычный останов.
    lifecycle_.beginStop();
    applyStop(port_.stop());

    switch (lifecycle_.state()) {
        case ServerState::Stopped:
            return ShutdownOutcome::Clean;
        case ServerState::StopFailed:
            return ShutdownOutcome::Terminal;
        default:
            return ShutdownOutcome::Retry;
    }
}

void ServerController::applyStop(StopOutcome outcome) noexcept
{
    switch (outcome) {
        case StopOutcome::Confirmed:
            lifecycle_.stopSucceeded();
            retry_pending_ = false;
            break;
        case StopOutcome::NotConfirmedRetryable:
            lifecycle_.stopUnconfirmed(true);
            retry_pending_ = lifecycle_.state() == ServerState::Stopping;
            break;
        case StopOutcome::NotConfirmedTerminal:
            lifecycle_.stopUnconfirmed(false);
            retry_pending_ = false;
            break;
    }
}

}  // namespace web
