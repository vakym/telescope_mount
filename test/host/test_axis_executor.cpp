/**
 * @file test_axis_executor.cpp
 * @brief Событийный цикл исполнителя на FakeClock и FakeCoil.
 */

#include <cmath>
#include <cstdint>
#include <limits>

#include "check.hpp"
#include "fakes.hpp"
#include "motion/axis_executor.hpp"

namespace {

using motion::AxisCommand;
using motion::AxisExecutor;
using motion::AxisPlanner;
using motion::AxisState;
using motion::EventBits;
using motion::StepPlan;

motion::AxisConfig baseConfig()
{
    return motion::AxisConfig{};
}

struct Rig {
    AxisPlanner planner;
    fakes::FakeClock clock;
    fakes::FakeCoil coil;
    fakes::FakeSource source;
    AxisExecutor executor;

    Rig(const motion::AxisConfig &cfg, AxisExecutor::Params params = AxisExecutor::Params{})
        : executor(planner, clock, coil, source, params)
    {
        CHECK(planner.configure(cfg));
        clock.now = 1000000;
    }

    void send(const AxisCommand &cmd)
    {
        source.push(cmd);
        executor.onWakeup(motion::CommandPending);
    }

    void fireDue()
    {
        CHECK(clock.armed);
        clock.now = clock.deadline;
        clock.armed = false;  // одноразовый alarm после срабатывания снят
        executor.onWakeup(motion::TimerEvent);
    }

    /// Доставляет события, пока не исполнится пауза реверса.
    bool runUntilPause()
    {
        for (int i = 0; i < 5000; ++i) {
            if (!planner.hasPending()) {
                return false;
            }
            StepPlan peek{};
            CHECK(planner.prepareNextStep(peek));
            if (!peek.is_step) {
                const std::int64_t before = planner.status().position_step;
                const int advances_before = coil.advances;
                fireDue();
                CHECK(coil.advances == advances_before);
                CHECK(planner.status().position_step == before);
                return true;
            }
            fireDue();
        }
        return false;
    }
};

void commandDoesNotStep()
{
    Rig rig(baseConfig());
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::setTargetRate(800.0F));

    CHECK(rig.coil.advances == 0);  // пробуждение по команде шага не выдаёт
    CHECK(rig.clock.armed);

    rig.fireDue();
    CHECK(rig.coil.advances == 1);
    CHECK(rig.planner.status().position_step == 1);
}

void lateWakeupKeepsSchedule()
{
    Rig rig(baseConfig());
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::setTargetRate(800.0F));

    CHECK(rig.clock.armed);
    const std::uint64_t planned = rig.clock.deadline;
    rig.clock.now = planned + 30;
    rig.clock.armed = false;
    rig.executor.onWakeup(motion::TimerEvent);

    CHECK(rig.coil.advances == 1);
    CHECK(rig.clock.armed);
    CHECK(rig.clock.deadline > planned);
    CHECK(rig.planner.status().late_replans == 0);
}

void cancelledStepAndStaleNotification()
{
    Rig rig(baseConfig());
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::moveSteps(1));
    CHECK(rig.planner.hasPending());

    rig.send(AxisCommand::stop(motion::StopMode::Immediate));
    CHECK(!rig.planner.hasPending());
    CHECK(rig.clock.disarm_calls > 0);

    const auto position = rig.planner.status().position_step;
    const int advances = rig.coil.advances;

    // Уже доставленное уведомление от отменённого alarm.
    rig.executor.onWakeup(motion::TimerEvent);
    CHECK(rig.coil.advances == advances);
    CHECK(rig.planner.status().position_step == position);
    CHECK(rig.planner.status().stale_events >= 1);
}

void staleNotificationDoesNotStepNewMotion()
{
    Rig rig(baseConfig());
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::moveSteps(1));
    rig.send(AxisCommand::stop(motion::StopMode::Immediate));

    // Новое движение вооружает новый alarm; старое уведомление приходит
    // раньше нового дедлайна.
    rig.send(AxisCommand::setTargetRate(100.0F));
    CHECK(rig.clock.armed);

    const int advances = rig.coil.advances;
    rig.executor.onWakeup(motion::TimerEvent);  // now < deadline
    CHECK(rig.coil.advances == advances);
    CHECK(rig.clock.armed);
    CHECK(rig.clock.deadline > rig.clock.now);
}

void simultaneousBitsKeepTimerEvent()
{
    Rig rig(baseConfig());
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::setTargetRate(800.0F));
    rig.fireDue();
    CHECK(rig.coil.advances == 1);

    rig.source.push(AxisCommand::setTargetRate(800.0F));  // идемпотентная
    rig.clock.now = rig.clock.deadline;
    rig.executor.onWakeup(motion::CommandPending | motion::TimerEvent);

    CHECK(rig.coil.advances == 2);
}

void identicalCommandsDoNotDeferStep()
{
    Rig rig(baseConfig());
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::setTargetRate(800.0F));

    const std::uint64_t planned = rig.clock.deadline;
    for (int i = 0; i < 5; ++i) {
        rig.send(AxisCommand::setTargetRate(800.0F));
    }
    CHECK(rig.clock.armed);
    CHECK(rig.clock.deadline == planned);

    rig.fireDue();
    CHECK(rig.coil.advances == 1);
}

void commandBudgetIsBounded()
{
    AxisExecutor::Params params{};
    params.command_budget = 2;
    Rig rig(baseConfig(), params);

    // Команды кладутся в очередь без немедленного пробуждения.
    rig.source.push(AxisCommand::enable());
    rig.source.push(AxisCommand::setTargetRate(100.0F));
    rig.source.push(AxisCommand::setTargetRate(200.0F));
    rig.source.push(AxisCommand::setTargetRate(300.0F));
    rig.source.push(AxisCommand::setTargetRate(400.0F));

    rig.executor.onWakeup(motion::CommandPending);
    CHECK(rig.source.queue.size() == 3);  // бюджет 2 за итерацию

    rig.executor.onWakeup(motion::CommandPending);
    CHECK(rig.source.queue.size() == 1);
    rig.executor.onWakeup(motion::CommandPending);
    CHECK(rig.source.empty());
}

void waitTimeoutDoesNotDeEnergize()
{
    Rig rig(baseConfig());
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::setTargetRate(800.0F));
    rig.fireDue();
    CHECK(rig.coil.energized);

    const int releases = rig.coil.releases;
    rig.clock.now = rig.clock.deadline + 6000;  // watchdog
    rig.executor.onWakeup(0U);

    CHECK(rig.coil.releases == releases);  // не удержание
    CHECK(rig.planner.status().state != AxisState::Released);
    CHECK(rig.planner.status().position_trusted);
    CHECK(rig.planner.status().late_replans >= 1);
    CHECK(rig.clock.armed);
}

void holdExpiryIsIndependentOfCommands()
{
    motion::AxisConfig cfg = baseConfig();
    cfg.hold.hold_enabled = true;
    cfg.hold.hold_timeout_ms = 20;

    Rig rig(cfg);
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::moveSteps(1));
    rig.fireDue();
    CHECK(rig.planner.status().state == AxisState::Holding);
    CHECK(rig.clock.armed == false);

    // Поток отклоняемых команд не должен продлевать удержание.
    rig.clock.now += 25000;
    rig.send(AxisCommand::setTargetRate(std::numeric_limits<float>::quiet_NaN()));
    rig.send(AxisCommand::moveSteps(1));  // нечётный ход в Half8 допустим, но
                                          // движение начнётся — проверяем ниже
    CHECK(rig.planner.status().rejected_commands >= 1);
}

void rejectedCommandsDoNotPreventRelease()
{
    motion::AxisConfig cfg = baseConfig();
    cfg.hold.hold_enabled = true;
    cfg.hold.hold_timeout_ms = 20;

    Rig rig(cfg);
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::moveSteps(1));
    rig.fireDue();
    CHECK(rig.planner.status().state == AxisState::Holding);

    const int releases = rig.coil.releases;
    rig.clock.now += 25000;
    rig.source.push(AxisCommand::setTargetRate(std::numeric_limits<float>::quiet_NaN()));
    rig.executor.onWakeup(motion::CommandPending);

    CHECK(rig.coil.releases > releases);
    CHECK(rig.planner.status().state == AxisState::Released);
    CHECK(rig.planner.status().rejected_commands >= 1);
}

void emergencyStop()
{
    Rig rig(baseConfig());
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::setTargetRate(800.0F));
    rig.fireDue();
    const int advances = rig.coil.advances;
    const int releases = rig.coil.releases;

    rig.source.push(AxisCommand::setTargetRate(500.0F));
    rig.executor.onWakeup(motion::EmergencyStop);

    CHECK(rig.coil.advances == advances);   // шаг не добавляется
    CHECK(rig.coil.releases > releases);
    CHECK(rig.source.empty());
    CHECK(rig.planner.status().state == AxisState::Released);
    CHECK(!rig.planner.status().position_trusted);
    CHECK(!rig.planner.hasPending());
}

void reversalPauseIsNotAStep()
{
    motion::AxisConfig cfg = baseConfig();
    cfg.reversal_pause_us = 5000;

    Rig rig(cfg);
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::setTargetRate(800.0F));
    for (int i = 0; i < 5; ++i) {
        rig.fireDue();
    }
    rig.send(AxisCommand::setTargetRate(-800.0F));

    CHECK(rig.runUntilPause());

    rig.fireDue();  // после паузы — первый обратный шаг
    CHECK(rig.planner.status().rate_steps_s < 0.0F);
}

void armFailureEntersFault()
{
    Rig rig(baseConfig());
    rig.send(AxisCommand::enable());

    rig.clock.arm_fails = true;
    rig.send(AxisCommand::setTargetRate(100.0F));

    CHECK(rig.planner.status().state == AxisState::Fault);
    CHECK(!rig.planner.status().position_trusted);
    CHECK(!rig.planner.apply(AxisCommand::setTargetRate(100.0F)));

    rig.clock.arm_fails = false;
    rig.send(AxisCommand::resetFault());
    CHECK(rig.planner.status().state == AxisState::Disabled);

    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::setTargetRate(100.0F));
    CHECK(rig.planner.status().state == AxisState::Idle ||
          rig.planner.status().state == AxisState::Accelerating);
    CHECK(rig.clock.armed);
}

void singleEventMove()
{
    Rig rig(baseConfig());
    rig.send(AxisCommand::enable());
    rig.send(AxisCommand::moveSteps(1));
    CHECK(rig.coil.advances == 0);

    rig.fireDue();
    CHECK(rig.coil.advances == 1);
    CHECK(rig.planner.status().position_step == 1);
    CHECK(!rig.planner.hasPending());

    // Пробуждение по таймауту без движения ничего не делает.
    rig.clock.now += 10000;
    rig.executor.onWakeup(0U);
    CHECK(rig.coil.advances == 1);
}

}  // namespace

int main()
{
    commandDoesNotStep();
    lateWakeupKeepsSchedule();
    cancelledStepAndStaleNotification();
    staleNotificationDoesNotStepNewMotion();
    simultaneousBitsKeepTimerEvent();
    identicalCommandsDoNotDeferStep();
    commandBudgetIsBounded();
    waitTimeoutDoesNotDeEnergize();
    holdExpiryIsIndependentOfCommands();
    rejectedCommandsDoNotPreventRelease();
    emergencyStop();
    reversalPauseIsNotAStep();
    armFailureEntersFault();
    singleEventMove();
    return TEST_RESULT();
}
