/**
 * @file test_retry_gate.cpp
 * @brief Host-тесты вычисления интервала между повторными попытками.
 *
 * Проверяется главное свойство: каждая выполненная неудачная попытка заново
 * отсчитывает полный интервал, поэтому повторы не учащаются до периода
 * супервизора.
 */

#include <cstdint>

#include "check.hpp"
#include "web/retry_gate.hpp"

namespace {

void testNotArmedAllowsAttempt()
{
    web::RetryGate gate;
    CHECK(gate.elapsed(0, 500));
    CHECK(gate.elapsed(1000, 500));
    CHECK(gate.attempts() == 0);
}

void testIntervalCountedFromAttemptEnd()
{
    web::RetryGate gate;
    gate.noteAttempt(100);
    CHECK(!gate.elapsed(100, 500));
    CHECK(!gate.elapsed(599, 500));
    CHECK(gate.elapsed(600, 500));
}

void testThreeFailuresKeepFullInterval()
{
    web::RetryGate gate;
    const std::uint32_t interval = 500;
    std::uint32_t attempt_times[3] = {0, 0, 0};
    std::uint32_t now = 0;

    // Попытка №1 завершилась в момент 0.
    gate.noteAttempt(now);
    attempt_times[0] = now;
    now += 250;
    CHECK(!gate.elapsed(now, interval));
    now += 250;  // 500
    CHECK(gate.elapsed(now, interval));

    // Попытка №2 завершилась в 500: интервал считается заново.
    gate.noteAttempt(now);
    attempt_times[1] = now;
    now += 250;  // 750 — прежний дефект разрешал попытку здесь
    CHECK(!gate.elapsed(now, interval));
    now += 250;  // 1000
    CHECK(gate.elapsed(now, interval));

    // Попытка №3 завершилась в 1000.
    gate.noteAttempt(now);
    attempt_times[2] = now;
    now += 250;  // 1250
    CHECK(!gate.elapsed(now, interval));
    now += 250;  // 1500
    CHECK(gate.elapsed(now, interval));

    CHECK(attempt_times[1] - attempt_times[0] == interval);
    CHECK(attempt_times[2] - attempt_times[1] == interval);
    CHECK(gate.attempts() == 3);

    gate.clear();
    CHECK(!gate.armed());
    CHECK(gate.elapsed(now, interval));
}

void testZeroTickIsValidTime()
{
    web::RetryGate gate;
    gate.noteAttempt(0);
    CHECK(gate.armed());
    CHECK(!gate.elapsed(0, 300));
    CHECK(!gate.elapsed(299, 300));
    CHECK(gate.elapsed(300, 300));
}

void testTickCounterWrap()
{
    web::RetryGate gate;
    gate.noteAttempt(0xFFFFFF00U);
    CHECK(!gate.elapsed(0x00000050U, 0x200));  // 336 тиков < 512
    CHECK(gate.elapsed(0x00000100U, 0x200));    // 512 тиков
}

}  // namespace

int main()
{
    testNotArmedAllowsAttempt();
    testIntervalCountedFromAttemptEnd();
    testThreeFailuresKeepFullInterval();
    testZeroTickIsValidTime();
    testTickCounterWrap();
    return TEST_RESULT();
}
