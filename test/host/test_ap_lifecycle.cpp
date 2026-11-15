/**
 * @file test_ap_lifecycle.cpp
 * @brief Проверки координатора: операции, поздние события, ресинхронизация.
 */

#include <cstdint>

#include "check.hpp"
#include "wifi/ap_lifecycle.hpp"

namespace {

using wifi::Action;
using wifi::ApCoordinator;
using wifi::Lifecycle;
using wifi::Operation;
using wifi::RadioState;

void normalToggleCycle()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    CHECK(coordinator.lifecycle() == Lifecycle::Starting);
    CHECK(coordinator.radioState() == RadioState::Unknown);

    coordinator.startupSucceeded(true);
    CHECK(coordinator.lifecycle() == Lifecycle::Running);
    CHECK(coordinator.radioState() == RadioState::Running);

    CHECK(coordinator.beginToggleOperation() == Action::StopRadio);
    CHECK(coordinator.operation() == Operation::Stopping);
    coordinator.onApStopped();
    CHECK(coordinator.radioState() == RadioState::Stopped);
    CHECK(coordinator.operation() == Operation::None);
    CHECK(coordinator.stops == 1);

    CHECK(coordinator.beginToggleOperation() == Action::StartRadio);
    coordinator.onApStarted();
    CHECK(coordinator.radioState() == RadioState::Running);
    CHECK(coordinator.operation() == Operation::None);
    CHECK(coordinator.starts == 1);
}

void startOnBootDisabled()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(false);
    CHECK(coordinator.radioState() == RadioState::Stopped);
    CHECK(coordinator.beginToggleOperation() == Action::StartRadio);
}

void operationBusy()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);
    CHECK(coordinator.beginToggleOperation() == Action::StopRadio);
    CHECK(coordinator.beginToggleOperation() == Action::None);
    CHECK(coordinator.operation() == Operation::Stopping);
}

void unknownBlocksToggle()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);
    coordinator.onOperationTimeout();
    CHECK(coordinator.radioState() == RadioState::Unknown);
    CHECK(coordinator.beginToggleOperation() == Action::None);
    CHECK(coordinator.operation() == Operation::None);
    CHECK(coordinator.event_timeouts == 1);
}

void resyncAfterTimeout()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);
    CHECK(coordinator.beginToggleOperation() == Action::StopRadio);
    coordinator.onOperationTimeout();
    CHECK(coordinator.radioState() == RadioState::Unknown);

    CHECK(coordinator.beginResync() == Action::ResyncStop);
    CHECK(coordinator.operation() == Operation::Resyncing);
    CHECK(coordinator.resyncs == 1);

    coordinator.onApStopped();
    CHECK(coordinator.radioState() == RadioState::Stopped);
    CHECK(coordinator.operation() == Operation::None);
}

void resyncFromExternalCause()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);
    (void)coordinator.beginToggleOperation();
    coordinator.onOperationTimeout();

    // Ожидающих заявок нет: поводом служит внешний запрос.
    CHECK(coordinator.beginResync() == Action::ResyncStop);
    coordinator.onAlreadyStopped();
    CHECK(coordinator.radioState() == RadioState::Stopped);
    CHECK(coordinator.operation() == Operation::None);
    // Автоматической новой попытки нет: счётчик не растёт сам.
    CHECK(coordinator.resyncs == 1);

    // Новый повод — новая попытка.
    CHECK(coordinator.beginResync() == Action::ResyncStop);
    CHECK(coordinator.resyncs == 2);
}

void resyncTimeoutDoesNotRecurse()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);
    (void)coordinator.beginToggleOperation();
    coordinator.onOperationTimeout();

    CHECK(coordinator.beginResync() == Action::ResyncStop);
    coordinator.onOperationTimeout();
    CHECK(coordinator.radioState() == RadioState::Unknown);
    CHECK(coordinator.operation() == Operation::None);
    CHECK(coordinator.resyncs == 1);
    CHECK(coordinator.event_timeouts == 2);
}

void immediateErrorGivesUnknown()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);
    CHECK(coordinator.beginToggleOperation() == Action::StopRadio);

    coordinator.onOperationFailed();
    CHECK(coordinator.operation() == Operation::None);
    CHECK(coordinator.radioState() == RadioState::Unknown);
    CHECK(coordinator.operation_errors == 1);
    CHECK(coordinator.event_timeouts == 0);
}

void alreadyStopped()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);
    (void)coordinator.beginToggleOperation();
    coordinator.onOperationTimeout();
    coordinator.onAlreadyStopped();
    CHECK(coordinator.radioState() == RadioState::Stopped);
    CHECK(coordinator.operation() == Operation::None);
}

void lateStartDuringResync()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);
    (void)coordinator.beginToggleOperation();

    coordinator.onOperationTimeout();
    CHECK(coordinator.radioState() == RadioState::Unknown);
    CHECK(coordinator.beginResync() == Action::ResyncStop);

    // Поздний AP_START меняет наблюдаемое радио, но не завершает
    // ресинхронизацию: подтверждается только остановка.
    coordinator.onApStarted();
    CHECK(coordinator.radioState() == RadioState::Running);
    CHECK(coordinator.operation() == Operation::Resyncing);

    coordinator.onApStopped();
    CHECK(coordinator.radioState() == RadioState::Stopped);
    CHECK(coordinator.operation() == Operation::None);
}

void oppositeEventsDoNotComplete()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(false);
    CHECK(coordinator.beginToggleOperation() == Action::StartRadio);

    // AP_STOP при операции Starting не завершает запуск.
    coordinator.onApStopped();
    CHECK(coordinator.operation() == Operation::Starting);
    coordinator.onApStarted();
    CHECK(coordinator.operation() == Operation::None);

    CHECK(coordinator.beginToggleOperation() == Action::StopRadio);
    // AP_START при операции Stopping не завершает остановку.
    coordinator.onApStarted();
    CHECK(coordinator.operation() == Operation::Stopping);
    coordinator.onApStopped();
    CHECK(coordinator.operation() == Operation::None);
}

void duplicatesDoNotDoubleCount()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    // Стартуем с подтверждённого Stopped, чтобы первое событие было
    // переходом, а не дубликатом состояния.
    coordinator.startupSucceeded(false);
    CHECK(coordinator.starts == 0);

    coordinator.onApStarted();
    CHECK(coordinator.starts == 1);
    coordinator.onApStarted();
    CHECK(coordinator.starts == 1);

    coordinator.onApStopped();
    CHECK(coordinator.stops == 1);
    coordinator.onApStopped();
    CHECK(coordinator.stops == 1);
}

void stationsCounter()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);

    coordinator.onStaConnected();
    coordinator.onStaConnected();
    CHECK(coordinator.stations() == 2);

    coordinator.onStaDisconnected();
    coordinator.onStaDisconnected();
    coordinator.onStaDisconnected();
    CHECK(coordinator.stations() == 0);

    coordinator.onStaConnected();
    coordinator.onApStopped();
    CHECK(coordinator.stations() == 0);
}

void shutdownCycle()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);

    CHECK(coordinator.beginShutdown());
    CHECK(coordinator.lifecycle() == Lifecycle::Stopping);
    CHECK(!coordinator.beginShutdown());
    CHECK(coordinator.lifecycle() == Lifecycle::Stopping);

    coordinator.finishShutdown();
    CHECK(coordinator.lifecycle() == Lifecycle::Stopped);
    CHECK(coordinator.radioState() == RadioState::Unknown);

    // Повторный успешный запуск после остановки.
    coordinator.beginStartup();
    coordinator.startupSucceeded(true);
    CHECK(coordinator.lifecycle() == Lifecycle::Running);
    CHECK(coordinator.radioState() == RadioState::Running);
}

void startupFailureThenRetry()
{
    ApCoordinator coordinator;
    coordinator.beginStartup();
    coordinator.startupFailed();
    CHECK(coordinator.lifecycle() == Lifecycle::Stopped);
    CHECK(coordinator.radioState() == RadioState::Unknown);

    coordinator.beginStartup();
    coordinator.startupSucceeded(true);
    CHECK(coordinator.lifecycle() == Lifecycle::Running);
}

}  // namespace

int main()
{
    normalToggleCycle();
    startOnBootDisabled();
    operationBusy();
    unknownBlocksToggle();
    resyncAfterTimeout();
    resyncFromExternalCause();
    resyncTimeoutDoesNotRecurse();
    immediateErrorGivesUnknown();
    alreadyStopped();
    lateStartDuringResync();
    oppositeEventsDoNotComplete();
    duplicatesDoNotDoubleCount();
    stationsCounter();
    shutdownCycle();
    startupFailureThenRetry();
    return TEST_RESULT();
}
