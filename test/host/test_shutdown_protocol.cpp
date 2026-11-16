/**
 * @file test_shutdown_protocol.cpp
 * @brief Host-тесты протокола завершения задачи-супервизора.
 *
 * Главное свойство: уведомление разрешено только на первый запрос, поэтому
 * повторный stop() после таймаута не может разбудить уже завершившуюся
 * задачу.
 */

#include "check.hpp"
#include "web/shutdown_protocol.hpp"

namespace {

using web::ShutdownProtocol;
using WaitAction = web::ShutdownProtocol::WaitAction;

void testFirstRequestNotifies()
{
    ShutdownProtocol protocol;
    CHECK(protocol.beginStop() == WaitAction::NotifyThenWait);
    CHECK(protocol.requests() == 1);
    CHECK(!protocol.stopped());
}

void testRetryAfterTimeoutNeverNotifiesAgain()
{
    ShutdownProtocol protocol;
    CHECK(protocol.beginStop() == WaitAction::NotifyThenWait);

    protocol.onWaitTimeout();
    CHECK(protocol.beginStop() == WaitAction::WaitOnly);
    protocol.onWaitTimeout();
    CHECK(protocol.beginStop() == WaitAction::WaitOnly);

    CHECK(protocol.timeouts() == 2);
    CHECK(protocol.requests() == 1);  // уведомление выдавалось ровно один раз
    CHECK(!protocol.stopped());       // таймаут не означает завершение
}

void testTimeoutThenLateCompletion()
{
    ShutdownProtocol protocol;
    CHECK(protocol.beginStop() == WaitAction::NotifyThenWait);
    protocol.onWaitTimeout();

    // Позднее завершение после таймаута: следующий вызов видит готовый итог.
    protocol.onCompleted();
    CHECK(protocol.stopped());
    CHECK(protocol.beginStop() == WaitAction::Completed);
    CHECK(protocol.requests() == 1);
}

void testResetForNewStart()
{
    ShutdownProtocol protocol;
    CHECK(protocol.beginStop() == WaitAction::NotifyThenWait);
    protocol.onCompleted();

    protocol.reset();
    CHECK(!protocol.stopped());
    CHECK(protocol.beginStop() == WaitAction::NotifyThenWait);
    CHECK(protocol.requests() == 2);
    CHECK(protocol.completions() == 1);
}

void testCompletedIsTerminalUntilReset()
{
    ShutdownProtocol protocol;
    CHECK(protocol.beginStop() == WaitAction::NotifyThenWait);
    protocol.onCompleted();
    CHECK(protocol.beginStop() == WaitAction::Completed);
    CHECK(protocol.beginStop() == WaitAction::Completed);
    CHECK(protocol.requests() == 1);
}

}  // namespace

int main()
{
    testFirstRequestNotifies();
    testRetryAfterTimeoutNeverNotifiesAgain();
    testTimeoutThenLateCompletion();
    testResetForNewStart();
    testCompletedIsTerminalUntilReset();
    return TEST_RESULT();
}
