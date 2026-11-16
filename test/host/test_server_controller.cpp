/**
 * @file test_server_controller.cpp
 * @brief Host-тесты обёртки сервиса: владение, останов и повторный запуск.
 *
 * Проверяется то, что не видно в чистом автомате: применение результата
 * реального сервера, финализация через те же правила и запрет уничтожения
 * владельца при неподтверждённом останове.
 */

#include "check.hpp"
#include "web/server_controller.hpp"

namespace {

using web::ServerAction;
using web::ServerController;
using web::ServerPort;
using web::ServerState;
using web::ShutdownOutcome;
using web::StartOutcome;
using web::StopOutcome;

class FakePort final : public ServerPort {
public:
    StartOutcome start_result = StartOutcome::Started;
    StopOutcome stop_result = StopOutcome::Confirmed;

    int starts = 0;
    int stops = 0;
    bool server = false;

    StartOutcome start() noexcept override
    {
        ++starts;
        if (start_result == StartOutcome::Started ||
            start_result == StartOutcome::FailedServerLiveRetryable ||
            start_result == StartOutcome::FailedServerLiveTerminal) {
            server = true;
        }
        return start_result;
    }

    StopOutcome stop() noexcept override
    {
        ++stops;
        if (stop_result == StopOutcome::Confirmed) {
            server = false;
        }
        return stop_result;
    }

    bool hasServer() const noexcept override { return server; }
};

void testStartAndRestartAfterConfirmedStop()
{
    FakePort port;
    ServerController controller(port);

    CHECK(controller.tick(true, false, true) == ServerAction::Start);
    CHECK(controller.state() == ServerState::Running);
    CHECK(port.server);

    // Обычный останов по выключению AP.
    CHECK(controller.tick(false, false, false) == ServerAction::Stop);
    CHECK(controller.state() == ServerState::Stopped);
    CHECK(!port.server);

    // Повторный запуск после подтверждённого останова обязан работать.
    CHECK(controller.tick(true, false, true) == ServerAction::Start);
    CHECK(controller.state() == ServerState::Running);
    CHECK(port.starts == 2);
}

void testShutdownWhileRunning()
{
    FakePort port;
    ServerController controller(port);
    (void)controller.tick(true, false, true);
    CHECK(controller.state() == ServerState::Running);

    CHECK(controller.shutdownOnce() == ShutdownOutcome::Clean);
    CHECK(controller.state() == ServerState::Stopped);
    CHECK(!port.server);
    CHECK(port.stops == 1);
}

void testShutdownRetryAfterUnconfirmedStop()
{
    FakePort port;
    port.stop_result = StopOutcome::NotConfirmedRetryable;
    ServerController controller(port);
    (void)controller.tick(true, false, true);

    CHECK(controller.shutdownOnce() == ShutdownOutcome::Retry);
    CHECK(controller.state() == ServerState::Stopping);
    CHECK(controller.retryPending());
    CHECK(port.server);  // сервер жив, владение сохранено

    // Повтор не выдаётся без истечения интервала, но владение не потеряно.
    CHECK(controller.shutdownOnce() == ShutdownOutcome::Retry);
    CHECK(port.server);

    port.stop_result = StopOutcome::Confirmed;
    CHECK(controller.shutdownOnce() == ShutdownOutcome::Clean);
    CHECK(controller.state() == ServerState::Stopped);
    CHECK(!port.server);
    CHECK(!controller.retryPending());
}

void testShutdownTerminalKeepsOwnership()
{
    FakePort port;
    port.stop_result = StopOutcome::NotConfirmedTerminal;
    ServerController controller(port);
    (void)controller.tick(true, false, true);

    CHECK(controller.shutdownOnce() == ShutdownOutcome::Terminal);
    CHECK(controller.state() == ServerState::StopFailed);
    CHECK(port.server);

    // Повтор шага финализации не выдаёт Start и не освобождает владение.
    CHECK(controller.shutdownOnce() == ShutdownOutcome::Terminal);
    CHECK(controller.state() == ServerState::StopFailed);
    CHECK(controller.tick(true, false, true) == ServerAction::None);
    CHECK(port.starts == 1);
}

void testStartFailureRetryGate()
{
    FakePort port;
    port.start_result = StartOutcome::Failed;
    ServerController controller(port);

    CHECK(controller.tick(true, false, true) == ServerAction::Start);
    CHECK(controller.state() == ServerState::StartFailed);
    CHECK(controller.retryPending());
    CHECK(!port.server);

    CHECK(controller.tick(true, false, false) == ServerAction::None);
    CHECK(controller.tick(true, false, true) == ServerAction::Start);
    CHECK(controller.state() == ServerState::StartFailed);
    CHECK(port.starts == 2);

    // После неудачи откат уже подтверждён: владения нет.
    CHECK(controller.shutdownOnce() == ShutdownOutcome::Clean);
}

void testPartialStartWithLiveServer()
{
    FakePort port;
    port.start_result = StartOutcome::FailedServerLiveRetryable;
    ServerController controller(port);

    CHECK(controller.tick(true, false, true) == ServerAction::Start);
    CHECK(controller.state() == ServerState::Stopping);
    CHECK(controller.retryPending());
    CHECK(port.server);

    port.stop_result = StopOutcome::Confirmed;
    CHECK(controller.shutdownOnce() == ShutdownOutcome::Clean);
    CHECK(controller.state() == ServerState::Stopped);
    CHECK(!port.server);
}

void testPartialStartTerminal()
{
    FakePort port;
    port.start_result = StartOutcome::FailedServerLiveTerminal;
    ServerController controller(port);

    CHECK(controller.tick(true, false, true) == ServerAction::Start);
    CHECK(controller.state() == ServerState::StopFailed);
    CHECK(controller.tick(true, false, true) == ServerAction::None);
    CHECK(controller.shutdownOnce() == ShutdownOutcome::Terminal);
}

void testNoAccessPointNoStart()
{
    FakePort port;
    ServerController controller(port);
    CHECK(controller.tick(false, false, true) == ServerAction::None);
    CHECK(port.starts == 0);
    // Финализация без сервера не требует останова.
    CHECK(controller.shutdownOnce() == ShutdownOutcome::Clean);
    CHECK(port.stops == 0);
}

}  // namespace

int main()
{
    testStartAndRestartAfterConfirmedStop();
    testShutdownWhileRunning();
    testShutdownRetryAfterUnconfirmedStop();
    testShutdownTerminalKeepsOwnership();
    testStartFailureRetryGate();
    testPartialStartWithLiveServer();
    testPartialStartTerminal();
    testNoAccessPointNoStart();
    return TEST_RESULT();
}
