/**
 * @file test_axis_planner.cpp
 * @brief Профиль, двухфазный контракт и политика состояний AxisPlanner.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "check.hpp"
#include "motion/axis_planner.hpp"

namespace {

using motion::AxisPlanner;
using motion::AxisCommand;
using motion::AxisState;
using motion::Direction;
using motion::StepPattern;
using motion::StepPlan;

motion::AxisConfig baseConfig(StepPattern pattern = StepPattern::Half8)
{
    motion::AxisConfig cfg;
    cfg.pattern = pattern;
    return cfg;
}

struct Rig {
    AxisPlanner planner;

    explicit Rig(const motion::AxisConfig &cfg)
    {
        CHECK(planner.configure(cfg));
        CHECK(planner.apply(AxisCommand::enable()));
    }

    bool step(StepPlan *out = nullptr)
    {
        StepPlan plan{};
        if (!planner.prepareNextStep(plan)) {
            return false;
        }
        if (out != nullptr) {
            *out = plan;
        }
        planner.commitPending();
        return true;
    }
};

/// Число шагов и суммарное время до достижения скорости >= target.
int accelerateTo(AxisPlanner &planner, float target, std::uint64_t *time_us, double *max_rate)
{
    int steps = 0;
    std::uint64_t total = 0;
    double vmax = 0.0;
    float previous = 0.0F;
    bool monotonic = true;

    StepPlan plan{};
    while (steps < 5000 && planner.prepareNextStep(plan)) {
        planner.commitPending();
        total += plan.interval_us;
        ++steps;
        const float rate = std::fabs(planner.status().rate_steps_s);
        if (rate + 1e-3F < previous) {
            monotonic = false;
        }
        previous = rate;
        vmax = std::max(vmax, static_cast<double>(rate));
        if (rate >= target - 0.1F) {
            break;
        }
    }

    CHECK(monotonic);
    if (time_us != nullptr) {
        *time_us = total;
    }
    if (max_rate != nullptr) {
        *max_rate = vmax;
    }
    return steps;
}

/// Прогон до полной остановки; возвращает число шагов, время и максимальный ход.
int runUntilIdle(AxisPlanner &planner, int limit, std::uint64_t *time_us, std::int64_t *max_position)
{
    int steps = 0;
    std::uint64_t total = 0;
    std::int64_t max_pos = planner.status().position_step;

    StepPlan plan{};
    while (steps < limit && planner.prepareNextStep(plan)) {
        planner.commitPending();
        total += plan.interval_us;
        if (plan.is_step) {
            ++steps;
        }
        max_pos = std::max(max_pos, planner.status().position_step);
    }

    if (time_us != nullptr) {
        *time_us = total;
    }
    if (max_position != nullptr) {
        *max_position = max_pos;
    }
    return steps;
}

void accelerationProfile()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(800.0F)));

    std::uint64_t time_us = 0;
    double vmax = 0.0;
    const int steps = accelerateTo(rig.planner, 800.0F, &time_us, &vmax);

    CHECK_NEAR(vmax, 800.0, 2.0);

    const double expected_steps = (800.0 * 800.0 - 100.0 * 100.0) / (2.0 * 800.0);
    CHECK_NEAR(steps, expected_steps, 2.0);

    const double expected_time = (800.0 - 100.0) / 800.0;
    CHECK_NEAR(static_cast<double>(time_us) / 1e6, expected_time, expected_time * 0.05);
}

void brakingDistance()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(800.0F)));
    (void)accelerateTo(rig.planner, 800.0F, nullptr, nullptr);

    CHECK(rig.planner.apply(AxisCommand::stop(motion::StopMode::Smooth)));
    const int steps = runUntilIdle(rig.planner, 5000, nullptr, nullptr);

    const double expected = 800.0 * 800.0 / (2.0 * 800.0);
    CHECK_NEAR(steps, expected, 2.0);
    CHECK(rig.planner.status().state == AxisState::Released);
}

void moveStepsExact()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::moveSteps(1000)));
    const int steps = runUntilIdle(rig.planner, 5000, nullptr, nullptr);

    CHECK(steps == 1000);
    CHECK(rig.planner.status().position_step == 1000);
}

void singleEventMove()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::moveSteps(1)));

    StepPlan plan{};
    CHECK(rig.planner.prepareNextStep(plan));
    CHECK(plan.is_step);
    CHECK(plan.direction == Direction::Forward);
    // Один полушаг: вход 100, выход 0, интервал 2*1/(100+0).
    CHECK_NEAR(plan.interval_us, 20000.0, 1.0);

    rig.planner.commitPending();
    CHECK(rig.planner.status().position_step == 1);
    CHECK(!rig.planner.prepareNextStep(plan));
    CHECK(rig.planner.status().state == AxisState::Released);
}

void fullStepMultiples()
{
    Rig odd(baseConfig(StepPattern::Full4Double));
    CHECK(!odd.planner.apply(AxisCommand::moveSteps(1)));
    CHECK(!odd.planner.apply(AxisCommand::moveSteps(-3)));
    CHECK(odd.planner.status().rejected_commands == 2);

    Rig even(baseConfig(StepPattern::Full4Double));
    CHECK(even.planner.apply(AxisCommand::moveSteps(2)));
    (void)runUntilIdle(even.planner, 100, nullptr, nullptr);
    CHECK(even.planner.status().position_step == 2);

    Rig four(baseConfig(StepPattern::Full4Double));
    CHECK(four.planner.apply(AxisCommand::moveSteps(-4)));
    (void)runUntilIdle(four.planner, 100, nullptr, nullptr);
    CHECK(four.planner.status().position_step == -4);
}

void slowTargetRate()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(1.0F)));

    StepPlan plan{};
    CHECK(rig.planner.prepareNextStep(plan));
    // 1 полушаг/с: интервал 1e6 мкс, а не start_rate.
    CHECK_NEAR(plan.interval_us, 1000000.0, 2.0);
    rig.planner.commitPending();
    CHECK_NEAR(rig.planner.status().rate_steps_s, 1.0, 0.01);
}

void reversalWithPause()
{
    motion::AxisConfig cfg = baseConfig();
    cfg.reversal_pause_us = 5000;
    Rig rig(cfg);

    CHECK(rig.planner.apply(AxisCommand::setTargetRate(800.0F)));
    (void)accelerateTo(rig.planner, 800.0F, nullptr, nullptr);
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(-800.0F)));

    bool saw_pause = false;
    std::int64_t position_before_pause = 0;

    StepPlan plan{};
    for (int i = 0; i < 5000; ++i) {
        CHECK(rig.planner.prepareNextStep(plan));
        if (!plan.is_step) {
            CHECK_NEAR(rig.planner.status().rate_steps_s, 0.0, 1e-3);
            position_before_pause = rig.planner.status().position_step;
            rig.planner.commitPending();
            CHECK(rig.planner.status().position_step == position_before_pause);
            saw_pause = true;
            break;
        }
        rig.planner.commitPending();
    }

    CHECK(saw_pause);
    CHECK(rig.planner.prepareNextStep(plan));
    CHECK(plan.is_step);
    CHECK(plan.direction == Direction::Backward);
}

void immediateStopCancelsStep()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(500.0F)));

    StepPlan plan{};
    CHECK(rig.planner.prepareNextStep(plan));
    const std::int64_t position_before = rig.planner.status().position_step;

    CHECK(rig.planner.apply(AxisCommand::stop(motion::StopMode::Immediate)));
    CHECK(!rig.planner.hasPending());
    CHECK(rig.planner.status().position_step == position_before);
    CHECK(!rig.planner.status().position_trusted);
    CHECK(rig.planner.status().state == AxisState::Released);
}

void targetReplacementOvershootsAndReturns()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(800.0F)));
    (void)accelerateTo(rig.planner, 800.0F, nullptr, nullptr);

    const std::int64_t start = rig.planner.status().position_step;
    const std::int64_t target = start + 2;
    CHECK(rig.planner.apply(AxisCommand::moveSteps(2)));

    std::int64_t max_position = start;
    (void)runUntilIdle(rig.planner, 20000, nullptr, &max_position);

    CHECK(rig.planner.status().position_step == target);
    CHECK_GT(max_position, target);  // цель была ближе тормозного пути
}

void pendingContract()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::moveSteps(4)));

    const std::int64_t before = rig.planner.status().position_step;
    StepPlan plan{};
    CHECK(rig.planner.prepareNextStep(plan));
    CHECK(rig.planner.status().position_step == before);

    rig.planner.cancelPendingStep();
    CHECK(rig.planner.status().position_step == before);
    CHECK(!rig.planner.hasPending());

    CHECK(rig.planner.prepareNextStep(plan));
    rig.planner.commitPending();
    CHECK(rig.planner.status().position_step == before + 1);
}

void holdPolicies()
{
    motion::AxisConfig holding = baseConfig();
    holding.hold.hold_enabled = true;
    holding.hold.hold_timeout_ms = 0;
    Rig rig_hold(holding);
    CHECK(rig_hold.planner.apply(AxisCommand::moveSteps(1)));
    (void)runUntilIdle(rig_hold.planner, 10, nullptr, nullptr);
    CHECK(rig_hold.planner.status().state == AxisState::Holding);

    Rig rig_release(baseConfig());
    CHECK(rig_release.planner.apply(AxisCommand::moveSteps(1)));
    (void)runUntilIdle(rig_release.planner, 10, nullptr, nullptr);
    CHECK(rig_release.planner.status().state == AxisState::Released);
}

void overflowRejected()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::moveSteps(1)));
    (void)runUntilIdle(rig.planner, 10, nullptr, nullptr);

    CHECK(!rig.planner.apply(AxisCommand::moveSteps(std::numeric_limits<std::int64_t>::max())));
    CHECK(rig.planner.status().rejected_commands == 1);
}

void invalidConfiguration()
{
    motion::AxisConfig cfg = baseConfig();
    cfg.accel_steps_s2 = 0.0F;
    CHECK(!motion::isValid(cfg));

    cfg = baseConfig();
    cfg.decel_steps_s2 = std::numeric_limits<float>::quiet_NaN();
    CHECK(!motion::isValid(cfg));

    cfg = baseConfig();
    cfg.max_rate_steps_s = std::numeric_limits<float>::infinity();
    CHECK(!motion::isValid(cfg));

    AxisPlanner planner;
    CHECK(!planner.configure(cfg));
    CHECK(!planner.apply(AxisCommand::enable()));
}

void idempotentTargetAndFault()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(400.0F)));
    const std::uint32_t key = rig.planner.planKey();
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(400.0F)));
    CHECK(rig.planner.planKey() == key);
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(500.0F)));
    CHECK(rig.planner.planKey() != key);

    rig.planner.enterFault(motion::FaultReason::DeadlineMiss);
    CHECK(rig.planner.status().state == AxisState::Fault);
    CHECK(!rig.planner.status().position_trusted);
    CHECK(!rig.planner.apply(AxisCommand::setTargetRate(100.0F)));
    CHECK(!rig.planner.apply(AxisCommand::enable()));

    CHECK(rig.planner.apply(AxisCommand::resetFault()));
    CHECK(rig.planner.status().state == AxisState::Disabled);
    CHECK(rig.planner.apply(AxisCommand::enable()));
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(100.0F)));
}

void rateDecreaseRespectsDecel()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(800.0F)));
    (void)accelerateTo(rig.planner, 800.0F, nullptr, nullptr);
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(100.0F)));

    float previous = std::fabs(rig.planner.status().rate_steps_s);
    StepPlan plan{};
    for (int i = 0; i < 5000 && rig.planner.prepareNextStep(plan); ++i) {
        rig.planner.commitPending();
        const float current = std::fabs(rig.planner.status().rate_steps_s);
        // Замедление не круче заданного decel и без скачков вниз.
        CHECK(current * current >= previous * previous - 2.0 * 800.0 - 1.0);
        CHECK(current <= previous + 1e-3F);
        previous = current;
        if (current <= 100.0F + 1e-2F) {
            break;
        }
    }
    CHECK_NEAR(previous, 100.0, 0.5);

    // Уставка удерживается, а не продолжает падать.
    CHECK(rig.planner.prepareNextStep(plan));
    rig.planner.commitPending();
    CHECK_NEAR(std::fabs(rig.planner.status().rate_steps_s), 100.0, 0.5);
}

void negativeRateDecreaseRespectsDecel()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(-800.0F)));
    (void)accelerateTo(rig.planner, 800.0F, nullptr, nullptr);
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(-100.0F)));

    float previous = std::fabs(rig.planner.status().rate_steps_s);
    StepPlan plan{};
    for (int i = 0; i < 5000 && rig.planner.prepareNextStep(plan); ++i) {
        rig.planner.commitPending();
        const float current = std::fabs(rig.planner.status().rate_steps_s);
        CHECK(rig.planner.status().rate_steps_s < 0.0F);
        CHECK(current * current >= previous * previous - 2.0 * 800.0 - 1.0);
        previous = current;
        if (current <= 100.0F + 1e-2F) {
            break;
        }
    }
    CHECK_NEAR(previous, 100.0, 0.5);
}

void replacementNearTargetRespectsDecel()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(150.0F)));
    (void)accelerateTo(rig.planner, 150.0F, nullptr, nullptr);

    const std::int64_t start = rig.planner.status().position_step;
    const std::int64_t target = start + 1;
    CHECK(rig.planner.apply(AxisCommand::moveSteps(1)));

    float previous = std::fabs(rig.planner.status().rate_steps_s);
    std::int64_t max_position = start;
    StepPlan plan{};
    for (int i = 0; i < 20000 && rig.planner.prepareNextStep(plan); ++i) {
        rig.planner.commitPending();
        const float current = std::fabs(rig.planner.status().rate_steps_s);
        if (plan.is_step) {
            CHECK(current * current >= previous * previous - 2.0 * 800.0 - 1.0);
        }
        previous = current;
        max_position = std::max(max_position, rig.planner.status().position_step);
    }

    CHECK(rig.planner.status().position_step == target);
    CHECK_GT(max_position, target);  // цель ближе тормозного пути
}

void decelBoundAcrossConfigs()
{
    struct Case {
        StepPattern pattern;
        float max_rate;
        float accel;
        float decel;
    };

    const Case cases[] = {
        {StepPattern::Half8, 400.0F, 400.0F, 400.0F},
        {StepPattern::Half8, 800.0F, 1600.0F, 800.0F},
        {StepPattern::Full4Double, 400.0F, 400.0F, 800.0F},
        {StepPattern::Full4Double, 800.0F, 800.0F, 1600.0F},
    };

    for (const Case &c : cases) {
        motion::AxisConfig cfg = baseConfig(c.pattern);
        cfg.max_rate_steps_s = c.max_rate;
        cfg.accel_steps_s2 = c.accel;
        cfg.decel_steps_s2 = c.decel;

        Rig rig(cfg);
        const float high = c.max_rate * 0.9F;
        CHECK(rig.planner.apply(AxisCommand::setTargetRate(high)));
        (void)accelerateTo(rig.planner, high, nullptr, nullptr);

        const float low = c.max_rate * 0.25F;
        CHECK(rig.planner.apply(AxisCommand::setTargetRate(low)));

        const double ds = static_cast<double>(motion::deltaHalfSteps(c.pattern));
        float previous = std::fabs(rig.planner.status().rate_steps_s);
        StepPlan plan{};

        for (int i = 0; i < 20000 && rig.planner.prepareNextStep(plan); ++i) {
            rig.planner.commitPending();
            const float current = std::fabs(rig.planner.status().rate_steps_s);
            const double limit_sq = previous * previous - 2.0 * c.decel * ds;
            CHECK(static_cast<double>(current) * current >= limit_sq * (1.0 - 1e-4) - 1e-6);
            previous = current;
            if (current <= low + 1e-2F) {
                break;
            }
        }

        CHECK_NEAR(previous, low, 0.5 + 0.01 * low);
    }
}

void holdPolicyDoesNotReplan()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(400.0F)));

    StepPlan plan{};
    CHECK(rig.planner.prepareNextStep(plan));
    const std::uint32_t key = rig.planner.planKey();

    motion::HoldPolicy policy{};
    policy.hold_enabled = true;
    policy.hold_timeout_ms = 100;
    CHECK(rig.planner.apply(AxisCommand::setHoldPolicy(policy)));

    CHECK(rig.planner.planKey() == key);  // alarm не отменяется
    CHECK(rig.planner.hasPending());
}

void moveStepsZeroIsSmoothStop()
{
    Rig rig(baseConfig());
    CHECK(rig.planner.apply(AxisCommand::setTargetRate(800.0F)));
    (void)accelerateTo(rig.planner, 800.0F, nullptr, nullptr);

    CHECK(rig.planner.apply(AxisCommand::moveSteps(0)));
    (void)runUntilIdle(rig.planner, 5000, nullptr, nullptr);
    CHECK_NEAR(rig.planner.status().rate_steps_s, 0.0, 1e-3);
}

}  // namespace

int main()
{
    accelerationProfile();
    brakingDistance();
    moveStepsExact();
    singleEventMove();
    fullStepMultiples();
    slowTargetRate();
    reversalWithPause();
    immediateStopCancelsStep();
    targetReplacementOvershootsAndReturns();
    pendingContract();
    holdPolicies();
    overflowRejected();
    invalidConfiguration();
    idempotentTargetAndFault();
    rateDecreaseRespectsDecel();
    negativeRateDecreaseRespectsDecel();
    replacementNearTargetRespectsDecel();
    decelBoundAcrossConfigs();
    holdPolicyDoesNotReplan();
    moveStepsZeroIsSmoothStop();
    return TEST_RESULT();
}
