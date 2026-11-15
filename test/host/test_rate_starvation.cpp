/**
 * @file test_rate_starvation.cpp
 * @brief Модель планировщика: пол минимальной скорости против голодания шагов.
 *
 * Каждая новая уставка отменяет подготовленный шаг и перевзводит расписание.
 * На малой скорости интервал шага больше периода управления, поэтому частые
 * изменения уставки «съедают» шаги. Пол min_run_rate = 40 полушагов/с даёт
 * интервал 25 мс — меньше периода управления.
 */

#include "check.hpp"
#include "fakes.hpp"
#include "motion/axis_executor.hpp"

namespace {

using motion::AxisCommand;
using motion::AxisExecutor;
using motion::AxisPlanner;

struct Rig {
    AxisPlanner planner;
    fakes::FakeClock clock;
    fakes::FakeCoil coil;
    fakes::FakeSource source;
    AxisExecutor executor;

    Rig() : executor(planner, clock, coil, source, AxisExecutor::Params{5000, 8, 1})
    {
        (void)planner.configure(motion::AxisConfig{});
        clock.now = 1'000'000;
    }

    void send(const AxisCommand &cmd)
    {
        source.push(cmd);
        executor.onWakeup(motion::CommandPending);
    }

    /// Модельное время: шаг 1 мс, сработавшие alarm исполняются.
    void runMs(int ms)
    {
        for (int i = 0; i < ms; ++i) {
            clock.now += 1000;
            if (clock.armed && clock.now >= clock.deadline) {
                clock.armed = false;  // одноразовый alarm
                executor.onWakeup(motion::TimerEvent);
            }
        }
    }
};

/// Шаги за total_ms при уставке rate; resend — новая уставка каждый период.
int stepsAt(float rate, int period_ms, int total_ms, bool resend)
{
    Rig rig;
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::setTargetRate(rate));
    int sent = 0;
    for (int elapsed = 0; elapsed < total_ms; elapsed += period_ms) {
        if (resend) {
            ++sent;
            // Незначимое изменение: планировщик всё равно видит новую команду.
            rig.send(AxisCommand::setTargetRate(rate + 0.01F * static_cast<float>(sent)));
        }
        rig.runMs(period_ms);
    }
    return rig.coil.advances;
}

void steadyAtMinimumRate()
{
    const int steps = stepsAt(40.0F, 50, 10'000, false);
    // 40 полушагов/с * 10 с = 400 шагов.
    CHECK(steps >= 350);
    CHECK(steps <= 450);
}

void steadyAtLowRateMoves()
{
    const int steps = stepsAt(5.0F, 50, 10'000, false);
    // 5 полушагов/с * 10 с = 50 шагов, интервал 200 мс.
    CHECK(steps >= 40);
    CHECK(steps <= 60);
}

void resendAtLowRateStarves()
{
    // Без пола минимальной скорости поток уставок каждые 50 мс отменяет шаги.
    const int steps = stepsAt(5.0F, 50, 10'000, true);
    CHECK(steps < 10);
}

void resendAtMinimumRateStillMoves()
{
    // С полом 40 полушагов/с интервал 25 мс меньше периода управления.
    const int steps = stepsAt(40.0F, 50, 10'000, true);
    CHECK(steps >= 120);
}

}  // namespace

int main()
{
    steadyAtMinimumRate();
    steadyAtLowRateMoves();
    resendAtLowRateStarves();
    resendAtMinimumRateStillMoves();
    return TEST_RESULT();
}
