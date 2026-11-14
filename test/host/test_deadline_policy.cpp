/**
 * @file test_deadline_policy.cpp
 * @brief Проверки classifyWakeup() и planNextStep().
 */

#include "check.hpp"
#include "motion/deadline_policy.hpp"

namespace {

using motion::DeadlineAction;
using motion::DeadlineMissPolicy;
using motion::StepDue;

void classify()
{
    // 1. Штатное опоздание 30 мкс не меняет решение.
    CHECK(motion::classifyWakeup(1030, 1000, true, 2000) == StepDue::ExecuteNow);

    // 2. Без ожидающего события уведомление устарело.
    CHECK(motion::classifyWakeup(1030, 1000, false, 2000) == StepDue::IgnoreStale);

    // 3. Раньше дедлайна шаг не исполняется ни при каком допуске.
    CHECK(motion::classifyWakeup(900, 1000, true, 2000) == StepDue::IgnoreSpurious);
    CHECK(motion::classifyWakeup(999, 1000, true, 2000) == StepDue::IgnoreSpurious);

    // 4. Опоздание больше порога — MissedTooFar.
    CHECK(motion::classifyWakeup(4000, 1000, true, 2000) == StepDue::MissedTooFar);
}

void planKeepsSchedule()
{
    // 5. Дедлайн из профиля позже обоих ограничений: график сохраняется.
    const auto d = motion::planNextStep(10000, 10500, 9000, 500, 600, 50, 200,
                                        DeadlineMissPolicy::Replan);
    CHECK(d.action == DeadlineAction::ArmAtDeadline);
    CHECK(d.deadline_us == 10500);
    CHECK(!d.count_correction);
    CHECK(!d.count_replan);
}

void planCorrection()
{
    // 6. Сдвиг в пределах допуска — коррекция, а не перепланирование.
    const auto d = motion::planNextStep(10000, 10450, 9900, 500, 600, 50, 200,
                                        DeadlineMissPolicy::Replan);
    CHECK(d.action == DeadlineAction::ArmAtDeadline);
    CHECK(d.count_correction);
    CHECK(!d.count_replan);
    CHECK(d.deadline_us == 10500);
}

void planReplan()
{
    // 7. Большой сдвиг при политике Replan: не раньше now + interval.
    const auto d = motion::planNextStep(10000, 10010, 10000, 5000, 20000, 50, 200,
                                        DeadlineMissPolicy::Replan);
    CHECK(d.action == DeadlineAction::ReplanFromNow);
    CHECK(d.count_replan);
    CHECK_GE(d.deadline_us, 15000);   // now + interval
    CHECK_GE(d.deadline_us, 30000);   // last_switch + min_phase
    CHECK_GT(d.deadline_us, 10000);   // всегда в будущем
}

void planFault()
{
    // 8. Та же ситуация при политике Fault.
    const auto d = motion::planNextStep(10000, 10010, 10000, 5000, 20000, 50, 200,
                                        DeadlineMissPolicy::Fault);
    CHECK(d.action == DeadlineAction::Fault);
}

void planRespectsBothMinimums()
{
    // 9. Оба ограничения соблюдаются одновременно.
    const auto d = motion::planNextStep(10000, 10010, 9950, 200, 600, 50, 0,
                                        DeadlineMissPolicy::Replan);
    CHECK_GE(d.deadline_us, 10050);   // now + min_alarm_lead
    CHECK_GE(d.deadline_us, 10550);   // last_switch + min_phase_interval
}

void planDeadlineInFuture()
{
    // 10. Ранний next_deadline подтягивается к now + lead.
    const auto d = motion::planNextStep(5000, 4000, 1000, 300, 600, 50, 100000,
                                        DeadlineMissPolicy::Replan);
    CHECK_GT(d.deadline_us, 5000);
    CHECK_GE(d.deadline_us, 5050);
}

void planIsPure()
{
    // 11. Повторный вызов с теми же входами даёт тот же результат.
    const auto a = motion::planNextStep(1000, 2000, 1000, 500, 600, 50, 200,
                                        DeadlineMissPolicy::Replan);
    const auto b = motion::planNextStep(1000, 2000, 1000, 500, 600, 50, 200,
                                        DeadlineMissPolicy::Replan);
    CHECK(a.action == b.action);
    CHECK(a.deadline_us == b.deadline_us);
    CHECK(a.count_correction == b.count_correction);
    CHECK(a.count_replan == b.count_replan);
}

}  // namespace

int main()
{
    classify();
    planKeepsSchedule();
    planCorrection();
    planReplan();
    planFault();
    planRespectsBothMinimums();
    planDeadlineInFuture();
    planIsPure();
    return TEST_RESULT();
}
