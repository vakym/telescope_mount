/**
 * @file test_toggle_admission.cpp
 * @brief Проверки единственного владельца принятых заявок.
 */

#include <cstdint>

#include "check.hpp"
#include "wifi/toggle_admission.hpp"

namespace {

using wifi::Lifecycle;
using wifi::RadioState;
using wifi::ToggleAdmission;

void acceptsWithinCapacity()
{
    ToggleAdmission admission;
    const auto first = admission.tryAdmit(Lifecycle::Running, RadioState::Stopped);
    CHECK(first.result == ToggleAdmission::Result::Accepted);
    CHECK(first.wake_task);
    CHECK(admission.pending() == 1);
    CHECK(admission.accepted.load() == 1);

    const auto second = admission.tryAdmit(Lifecycle::Running, RadioState::Running);
    CHECK(second.result == ToggleAdmission::Result::Accepted);
    CHECK(second.wake_task);
    CHECK(admission.pending() == 2);

    const auto third = admission.tryAdmit(Lifecycle::Running, RadioState::Running);
    CHECK(third.result == ToggleAdmission::Result::Full);
    CHECK(!third.wake_task);
    CHECK(admission.pending() == 2);
    CHECK(admission.rejected_full.load() == 1);
    CHECK(admission.accepted.load() == 2);
}

void takeOneByOne()
{
    ToggleAdmission admission;
    (void)admission.tryAdmit(Lifecycle::Running, RadioState::Stopped);
    (void)admission.tryAdmit(Lifecycle::Running, RadioState::Stopped);

    CHECK(admission.takeOne());
    CHECK(admission.pending() == 1);
    CHECK(admission.takeOne());
    CHECK(admission.pending() == 0);
    CHECK(!admission.takeOne());

    // После освобождения вместимости приём снова работает.
    const auto again = admission.tryAdmit(Lifecycle::Running, RadioState::Stopped);
    CHECK(again.result == ToggleAdmission::Result::Accepted);
    CHECK(admission.pending() == 1);
}

void notRunningRejected()
{
    ToggleAdmission admission;
    const auto outcome = admission.tryAdmit(Lifecycle::Stopped, RadioState::Stopped);
    CHECK(outcome.result == ToggleAdmission::Result::NotRunning);
    CHECK(!outcome.wake_task);
    CHECK(admission.pending() == 0);
    CHECK(admission.accepted.load() == 0);
    CHECK(admission.rejected_not_running.load() == 1);

    const auto starting = admission.tryAdmit(Lifecycle::Starting, RadioState::Unknown);
    CHECK(starting.result == ToggleAdmission::Result::NotRunning);
    CHECK(admission.rejected_not_running.load() == 2);
}

void unconfirmedWakesTask()
{
    ToggleAdmission admission;
    const auto outcome = admission.tryAdmit(Lifecycle::Running, RadioState::Unknown);
    CHECK(outcome.result == ToggleAdmission::Result::Unconfirmed);
    // Отказ, но задача обязана проснуться ради ресинхронизации.
    CHECK(outcome.wake_task);
    CHECK(admission.pending() == 0);
    CHECK(admission.accepted.load() == 0);
    CHECK(admission.rejected_unconfirmed.load() == 1);
}

void accountsCurrentRequest()
{
    ToggleAdmission admission;
    admission.accountExecuted();
    CHECK(admission.executed.load() == 1);

    admission.accountFailed(ToggleAdmission::DropReason::Timeout);
    CHECK(admission.failed_on_timeout.load() == 1);
    admission.accountFailed(ToggleAdmission::DropReason::Error);
    CHECK(admission.failed_on_error.load() == 1);
    admission.accountFailed(ToggleAdmission::DropReason::Shutdown);
    CHECK(admission.failed_on_shutdown.load() == 1);
}

void dropsOnlyWaiting()
{
    ToggleAdmission admission;
    (void)admission.tryAdmit(Lifecycle::Running, RadioState::Stopped);
    (void)admission.tryAdmit(Lifecycle::Running, RadioState::Stopped);
    CHECK(admission.takeOne());
    CHECK(admission.pending() == 1);

    // Отбрасывается только ожидающая заявка, извлечённая не затрагивается.
    CHECK(admission.dropPending(ToggleAdmission::DropReason::Timeout) == 1);
    CHECK(admission.pending() == 0);
    CHECK(admission.dropped_on_timeout.load() == 1);
    CHECK(admission.dropped_on_shutdown.load() == 0);
    CHECK(admission.dropped_on_error.load() == 0);

    // Извлечённая заявка учитывается отдельно как исполненная.
    admission.accountExecuted();
    CHECK(admission.accepted.load() == 2);
    CHECK(admission.executed.load() == 1);
}

void balanceAfterCompletion()
{
    ToggleAdmission admission;

    // Заявка A принята и исполнена.
    (void)admission.tryAdmit(Lifecycle::Running, RadioState::Stopped);
    CHECK(admission.takeOne());
    admission.accountExecuted();

    // Заявка B принята, заявка C принята и осталась ожидающей.
    (void)admission.tryAdmit(Lifecycle::Running, RadioState::Stopped);
    (void)admission.tryAdmit(Lifecycle::Running, RadioState::Stopped);
    CHECK(admission.takeOne());

    // Shutdown: ожидающая C отброшена, извлечённая B завершилась неудачей.
    CHECK(admission.dropPending(ToggleAdmission::DropReason::Shutdown) == 1);
    admission.accountFailed(ToggleAdmission::DropReason::Shutdown);

    CHECK(admission.pending() == 0);
    const std::uint32_t total = admission.executed.load() +
                                admission.failed_on_shutdown.load() +
                                admission.failed_on_timeout.load() +
                                admission.failed_on_error.load() +
                                admission.dropped_on_shutdown.load() +
                                admission.dropped_on_timeout.load() +
                                admission.dropped_on_error.load();
    CHECK(total == admission.accepted.load());
}

}  // namespace

int main()
{
    acceptsWithinCapacity();
    takeOneByOne();
    notRunningRejected();
    unconfirmedWakesTask();
    accountsCurrentRequest();
    dropsOnlyWaiting();
    balanceAfterCompletion();
    return TEST_RESULT();
}
