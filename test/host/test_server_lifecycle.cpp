/**
 * @file test_server_lifecycle.cpp
 * @brief Host-тесты автомата запуска и останова HTTP-сервера.
 *
 * Проверяется главное правило: `Stopped` и «забывание» handle допустимы
 * только после подтверждённого завершения; неподтверждённый останов
 * запрещает новый запуск.
 */

#include "check.hpp"
#include "web/server_lifecycle.hpp"

namespace {

using web::ServerAction;
using web::ServerLifecycle;
using web::ServerState;

void testNoAccessPointNoStart()
{
    ServerLifecycle lifecycle;
    CHECK(lifecycle.update(false, false, true) == ServerAction::None);
    CHECK(lifecycle.state() == ServerState::Stopped);
    CHECK(lifecycle.starts == 0);
}

void testSingleStart()
{
    ServerLifecycle lifecycle;
    CHECK(lifecycle.update(true, false, true) == ServerAction::Start);
    CHECK(lifecycle.state() == ServerState::Starting);
    CHECK(lifecycle.starts == 1);

    lifecycle.startSucceeded();
    CHECK(lifecycle.state() == ServerState::Running);
    CHECK(lifecycle.starts_confirmed == 1);

    // Повторные события AP не порождают второго запуска.
    CHECK(lifecycle.update(true, false, true) == ServerAction::None);
    CHECK(lifecycle.starts == 1);
}

void testStopOnAccessPointLoss()
{
    ServerLifecycle lifecycle;
    (void)lifecycle.update(true, false, true);
    lifecycle.startSucceeded();
    CHECK(lifecycle.update(false, false, false) == ServerAction::Stop);
    CHECK(lifecycle.state() == ServerState::Stopping);
    CHECK(lifecycle.stops == 1);
    lifecycle.stopSucceeded();
    CHECK(lifecycle.state() == ServerState::Stopped);
    CHECK(lifecycle.stops_confirmed == 1);
}

void testEarlyStopWhileStopping()
{
    ServerLifecycle lifecycle;
    (void)lifecycle.update(true, false, true);
    lifecycle.startSucceeded();
    CHECK(lifecycle.update(true, true, false) == ServerAction::Stop);
    CHECK(lifecycle.state() == ServerState::Stopping);
}

void testStartFailureRetryGate()
{
    ServerLifecycle lifecycle;
    CHECK(lifecycle.update(true, false, true) == ServerAction::Start);
    lifecycle.startFailed();
    CHECK(lifecycle.state() == ServerState::StartFailed);
    CHECK(lifecycle.start_failures == 1);

    CHECK(lifecycle.update(true, false, false) == ServerAction::None);
    CHECK(lifecycle.state() == ServerState::StartFailed);

    CHECK(lifecycle.update(true, false, true) == ServerAction::Start);
    CHECK(lifecycle.state() == ServerState::Starting);
    CHECK(lifecycle.starts == 2);

    lifecycle.startSucceeded();
    CHECK(lifecycle.state() == ServerState::Running);
}

void testConfirmedStopAllowsRestart()
{
    ServerLifecycle lifecycle;
    (void)lifecycle.update(true, false, true);
    lifecycle.startSucceeded();
    CHECK(lifecycle.update(false, false, false) == ServerAction::Stop);
    lifecycle.stopSucceeded();

    CHECK(lifecycle.update(true, false, true) == ServerAction::Start);
    CHECK(lifecycle.starts == 2);
}

void testUnconfirmedStopRetryable()
{
    ServerLifecycle lifecycle;
    (void)lifecycle.update(true, false, true);
    lifecycle.startSucceeded();
    CHECK(lifecycle.update(false, false, false) == ServerAction::Stop);

    lifecycle.stopUnconfirmed(true);
    CHECK(lifecycle.state() == ServerState::Stopping);
    CHECK(lifecycle.stop_unconfirmed == 1);
    CHECK(lifecycle.stops_confirmed == 0);

    // До истечения интервала повтора решение не выдаётся.
    CHECK(lifecycle.update(false, false, false) == ServerAction::None);
    CHECK(lifecycle.update(false, false, true) == ServerAction::Stop);
    CHECK(lifecycle.stops == 2);

    lifecycle.stopSucceeded();
    CHECK(lifecycle.state() == ServerState::Stopped);
    CHECK(lifecycle.stops_confirmed == 1);
}

void testUnconfirmedStopTerminal()
{
    ServerLifecycle lifecycle;
    (void)lifecycle.update(true, false, true);
    lifecycle.startSucceeded();
    CHECK(lifecycle.update(false, false, false) == ServerAction::Stop);

    lifecycle.stopUnconfirmed(false);
    CHECK(lifecycle.state() == ServerState::StopFailed);
    CHECK(lifecycle.stop_terminal == 1);

    // Новый запуск запрещён безусловно.
    CHECK(lifecycle.update(true, false, true) == ServerAction::None);
    CHECK(lifecycle.update(true, false, true) == ServerAction::None);
    CHECK(lifecycle.starts == 1);
}

void testPartialStartRollback()
{
    ServerLifecycle lifecycle;
    CHECK(lifecycle.update(true, false, true) == ServerAction::Start);
    // Регистрация маршрута не удалась, откат останова подтверждён.
    lifecycle.startFailed();
    CHECK(lifecycle.state() == ServerState::StartFailed);
    CHECK(lifecycle.starts_confirmed == 0);

    // Повтор разрешён только после интервала и только при работающем AP.
    CHECK(lifecycle.update(false, false, true) == ServerAction::None);
    CHECK(lifecycle.update(true, false, true) == ServerAction::Start);

    // Тот же сценарий, но откат останова не подтверждён.
    ServerLifecycle second;
    (void)second.update(true, false, true);
    second.startSucceeded();
    (void)second.update(false, false, false);
    second.stopUnconfirmed(false);
    CHECK(second.state() == ServerState::StopFailed);
    CHECK(second.update(true, false, true) == ServerAction::None);
}

void testStartFailedWithLiveServer()
{
    ServerLifecycle lifecycle;
    CHECK(lifecycle.update(true, false, true) == ServerAction::Start);
    // Регистрация маршрута не удалась, откат останова не подтверждён.
    lifecycle.startFailedWithLiveServer(true);
    CHECK(lifecycle.state() == ServerState::Stopping);
    CHECK(lifecycle.start_failures == 1);
    CHECK(lifecycle.stop_unconfirmed == 1);

    // Немедленный Start не выдаётся; повтор останова — по интервалу.
    CHECK(lifecycle.update(true, false, false) == ServerAction::None);
    CHECK(lifecycle.update(true, false, true) == ServerAction::Stop);
    lifecycle.stopSucceeded();
    CHECK(lifecycle.state() == ServerState::Stopped);
    CHECK(lifecycle.update(true, false, true) == ServerAction::Start);

    ServerLifecycle terminal;
    CHECK(terminal.update(true, false, true) == ServerAction::Start);
    terminal.startFailedWithLiveServer(false);
    CHECK(terminal.state() == ServerState::StopFailed);
    CHECK(terminal.stop_terminal == 1);
    CHECK(terminal.update(true, false, true) == ServerAction::None);
}

void testFastToggle()
{
    ServerLifecycle lifecycle;
    CHECK(lifecycle.update(true, false, true) == ServerAction::Start);
    lifecycle.startSucceeded();
    CHECK(lifecycle.update(false, false, true) == ServerAction::Stop);
    lifecycle.stopSucceeded();
    CHECK(lifecycle.update(true, false, true) == ServerAction::Start);
    lifecycle.startSucceeded();
    CHECK(lifecycle.state() == ServerState::Running);
    CHECK(lifecycle.starts == 2);
    CHECK(lifecycle.starts_confirmed == 2);
    CHECK(lifecycle.stops == 1);
    CHECK(lifecycle.stops_confirmed == 1);
}

}  // namespace

int main()
{
    testNoAccessPointNoStart();
    testSingleStart();
    testStopOnAccessPointLoss();
    testEarlyStopWhileStopping();
    testStartFailureRetryGate();
    testConfirmedStopAllowsRestart();
    testUnconfirmedStopRetryable();
    testUnconfirmedStopTerminal();
    testPartialStartRollback();
    testStartFailedWithLiveServer();
    testFastToggle();
    return TEST_RESULT();
}
